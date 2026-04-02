#!/usr/bin/env python3
"""
test_main_integration.py — Standalone tests for main.py orchestrator logic.

Tests the camera helper functions (_start_camera, _stop_camera, _trigger_camera),
the lock file API (write_lock, clear_lock), haversine_to_enu coordinate math,
and state machine transition guards — all without requiring ROS 2, a live FCU,
or physical hardware.

Strategy
--------
main.py imports rclpy at runtime inside MainNode.__init__, not at module level,
which means the module can be imported on any machine without ROS installed.
We import it via importlib (same pattern as test_camera_capture.py) and mock
only the objects that are actually called by the functions under test.

Test categories
---------------
1. TestLockFile          write_lock / clear_lock file I/O and JSON schema
2. TestHaversineToENU    coordinate conversion correctness and edge cases
3. TestCameraHelpers     _start_camera / _stop_camera / _trigger_camera
4. TestStateMachineGuards run_preflight_checks individual check paths (mocked)

Usage
-----
    python test_main_integration.py          # all tests
    python test_main_integration.py -v       # verbose
    python test_main_integration.py TestLockFile   # one class
"""

import importlib.util
import json
import math
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch, PropertyMock

# ── Import main.py without executing main() ───────────────────────────────────

_MAIN_PATH = Path(__file__).resolve().parent.parent / \
    "unitree_drone_mapper" / "main.py"

if not _MAIN_PATH.exists():
    _MAIN_PATH = Path(__file__).resolve().parents[1] / \
        "unitree_drone_mapper" / "main.py"

if not _MAIN_PATH.exists():
    print(f"[ERROR] Cannot locate main.py — expected at:\n  {_MAIN_PATH}")
    sys.exit(1)

spec = importlib.util.spec_from_file_location("dronepi_main", _MAIN_PATH)
_main_mod = importlib.util.module_from_spec(spec)

# Stub rclpy before the module body runs so the import at module scope
# (inside TYPE_CHECKING guard, if any) does not raise.
# main.py defers rclpy import to MainNode.__init__, so this is a safety net.
sys.modules.setdefault("rclpy", MagicMock())
sys.modules.setdefault("rclpy.node", MagicMock())
sys.modules.setdefault("rclpy.qos", MagicMock())
sys.modules.setdefault("mavros_msgs", MagicMock())
sys.modules.setdefault("mavros_msgs.msg", MagicMock())
sys.modules.setdefault("mavros_msgs.srv", MagicMock())
sys.modules.setdefault("sensor_msgs", MagicMock())
sys.modules.setdefault("sensor_msgs.msg", MagicMock())
sys.modules.setdefault("std_msgs", MagicMock())
sys.modules.setdefault("std_msgs.msg", MagicMock())
sys.modules.setdefault("geometry_msgs", MagicMock())
sys.modules.setdefault("geometry_msgs.msg", MagicMock())

spec.loader.exec_module(_main_mod)

# Aliases for the symbols under test
write_lock       = _main_mod.write_lock
clear_lock       = _main_mod.clear_lock
haversine_to_enu = _main_mod.haversine_to_enu
FlightMode       = _main_mod.FlightMode
MISSION_LOCK     = _main_mod.MISSION_LOCK
FLIGHT_DIR       = _main_mod.FLIGHT_DIR

# Camera helpers
_start_camera   = _main_mod._start_camera
_stop_camera    = _main_mod._stop_camera
_trigger_camera = _main_mod._trigger_camera


# ══════════════════════════════════════════════════════════════════════════════
# 1. Lock File
# ══════════════════════════════════════════════════════════════════════════════

class TestLockFile(unittest.TestCase):
    """write_lock() and clear_lock() file I/O and schema validation."""

    def setUp(self):
        """Redirect MISSION_LOCK to a temp file for isolation."""
        self._tmp_lock = Path(tempfile.mktemp(suffix=".lock"))
        self._orig_lock = _main_mod.MISSION_LOCK
        _main_mod.MISSION_LOCK = self._tmp_lock

    def tearDown(self):
        _main_mod.MISSION_LOCK = self._orig_lock
        self._tmp_lock.unlink(missing_ok=True)

    def test_write_lock_creates_file(self):
        write_lock("manual_scan")
        self.assertTrue(self._tmp_lock.exists())

    def test_write_lock_valid_json(self):
        write_lock("autonomous")
        payload = json.loads(self._tmp_lock.read_text())
        self.assertIsInstance(payload, dict)

    def test_write_lock_mode_field(self):
        write_lock("autonomous")
        payload = json.loads(self._tmp_lock.read_text())
        self.assertEqual(payload["mode"], "autonomous")

    def test_write_lock_started_at_present(self):
        write_lock("manual_scan")
        payload = json.loads(self._tmp_lock.read_text())
        self.assertIn("started_at", payload)

    def test_write_lock_extra_fields_merged(self):
        write_lock("autonomous", extra={"waypoint": 3, "session": "test"})
        payload = json.loads(self._tmp_lock.read_text())
        self.assertEqual(payload["waypoint"], 3)
        self.assertEqual(payload["session"], "test")

    def test_write_lock_overwrites_previous(self):
        write_lock("manual_scan")
        write_lock("autonomous")
        payload = json.loads(self._tmp_lock.read_text())
        self.assertEqual(payload["mode"], "autonomous")

    def test_clear_lock_removes_file(self):
        write_lock("manual_scan")
        self.assertTrue(self._tmp_lock.exists())
        clear_lock()
        self.assertFalse(self._tmp_lock.exists())

    def test_clear_lock_absent_is_safe(self):
        """clear_lock() on a non-existent file must not raise."""
        self.assertFalse(self._tmp_lock.exists())
        try:
            clear_lock()
        except Exception as exc:
            self.fail(f"clear_lock() raised on missing file: {exc}")

    def test_clear_lock_idempotent(self):
        """Calling clear_lock() twice must not raise."""
        write_lock("manual_scan")
        clear_lock()
        clear_lock()   # second call — file already gone


# ══════════════════════════════════════════════════════════════════════════════
# 2. Haversine ENU conversion
# ══════════════════════════════════════════════════════════════════════════════

class TestHaversineToENU(unittest.TestCase):
    """
    haversine_to_enu(lat, lon, alt, hlat, hlon, halt) → (east, north, up).

    Reference values computed independently using the WGS-84 flat-earth
    approximation:
        ΔN = ΔlatRad × R
        ΔE = ΔlonRad × R × cos(lat_mid)
        R  = 6 371 000 m
    """

    def test_identity_returns_zero(self):
        """Point at home → (0, 0, 0)."""
        e, n, u = haversine_to_enu(18.0, -67.0, 100.0,  18.0, -67.0, 100.0)
        self.assertAlmostEqual(e, 0.0, places=6)
        self.assertAlmostEqual(n, 0.0, places=6)
        self.assertAlmostEqual(u, 0.0, places=6)

    def test_altitude_difference(self):
        """Up component equals altitude delta exactly."""
        _, _, u = haversine_to_enu(18.0, -67.0, 115.0,  18.0, -67.0, 100.0)
        self.assertAlmostEqual(u, 15.0, places=6)

    def test_north_direction(self):
        """Moving north (positive Δlat) → positive north component."""
        _, n, _ = haversine_to_enu(18.01, -67.0, 0.0,  18.0, -67.0, 0.0)
        self.assertGreater(n, 0.0)

    def test_south_direction(self):
        """Moving south → negative north."""
        _, n, _ = haversine_to_enu(17.99, -67.0, 0.0,  18.0, -67.0, 0.0)
        self.assertLess(n, 0.0)

    def test_east_direction(self):
        """Moving east (positive Δlon) → positive east."""
        e, _, _ = haversine_to_enu(18.0, -66.99, 0.0,  18.0, -67.0, 0.0)
        self.assertGreater(e, 0.0)

    def test_west_direction(self):
        """Moving west → negative east."""
        e, _, _ = haversine_to_enu(18.0, -67.01, 0.0,  18.0, -67.0, 0.0)
        self.assertLess(e, 0.0)

    def test_1_degree_north_approx_111km(self):
        """1° latitude ≈ 111 195 m — within 10 m of reference."""
        _, n, _ = haversine_to_enu(19.0, -67.0, 0.0,  18.0, -67.0, 0.0)
        self.assertAlmostEqual(n, 111_195.0, delta=10.0)

    def test_longitude_scale_by_cosine(self):
        """
        East displacement at lat=60° must be ~half that at lat=0°
        for the same Δlon, because cos(60°)=0.5.
        """
        e_equator, _, _ = haversine_to_enu(0.0, 1.0, 0.0,  0.0, 0.0, 0.0)
        e_60deg,   _, _ = haversine_to_enu(60.0, 1.0, 0.0, 60.0, 0.0, 0.0)
        ratio = e_60deg / e_equator
        self.assertAlmostEqual(ratio, math.cos(math.radians(60.0)), places=4)

    def test_small_displacement_magnitude(self):
        """
        10m north + 10m east at ~18°N.
        Δlat for 10m ≈ 10/111195 degrees.
        Δlon for 10m at 18°N ≈ 10/(111195*cos(18°)) degrees.
        Result magnitude ≈ √(10²+10²) = 14.14m within 1m.
        """
        R     = 6_371_000.0
        dlat  = 10.0 / R
        dlon  = 10.0 / (R * math.cos(math.radians(18.0)))
        lat   = 18.0 + math.degrees(dlat)
        lon   = -67.0 + math.degrees(dlon)
        e, n, _ = haversine_to_enu(lat, lon, 0.0,  18.0, -67.0, 0.0)
        magnitude = math.sqrt(e**2 + n**2)
        self.assertAlmostEqual(magnitude, math.sqrt(200.0), delta=1.0)

    def test_return_type_is_tuple_of_three(self):
        result = haversine_to_enu(18.0, -67.0, 50.0,  18.0, -67.0, 50.0)
        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 3)


# ══════════════════════════════════════════════════════════════════════════════
# 3. Camera Helpers
# ══════════════════════════════════════════════════════════════════════════════

class _MockCameraCapture:
    """Minimal stand-in for CameraCapture used by _start_camera tests."""

    def __init__(self, will_start=True, session_id=None, **kwargs):
        self.will_start    = will_start
        self.session_id    = session_id
        self.output_dir    = Path("/mnt/ssd/flights") / (session_id or "test")
        self._frame_index  = 0
        self.started       = False
        self.stopped       = False
        self.triggers      = []   # list of context dicts posted via trigger()

    def start(self):
        self.started = True
        return self.will_start

    def stop(self):
        self.stopped = True

    def trigger(self, context):
        self.triggers.append(context)
        self._frame_index += 1
        # Return a fake path for a successful trigger
        return Path(f"/mnt/ssd/flights/test/frame_{self._frame_index:04d}.jpg")


class TestCameraHelpers(unittest.TestCase):
    """_start_camera, _stop_camera, _trigger_camera behaviour."""

    # ── _start_camera ─────────────────────────────────────────────────────────

    def test_start_camera_returns_instance_on_success(self):
        mock_cls = MagicMock(return_value=_MockCameraCapture(will_start=True))
        with patch.dict(sys.modules, {"camera_capture": MagicMock(CameraCapture=mock_cls)}):
            with patch.object(sys, "path", sys.path):
                # Make the import inside _start_camera succeed
                cam = self._start_with_mock_cls(_MockCameraCapture(will_start=True))
        self.assertIsNotNone(cam)

    def test_start_camera_returns_none_when_start_fails(self):
        cam = self._start_with_mock_cls(_MockCameraCapture(will_start=False))
        self.assertIsNone(cam)

    def test_start_camera_returns_none_on_import_error(self):
        """If camera_capture.py is missing, _start_camera must return None."""
        original_path = _main_mod.FLIGHT_DIR
        _main_mod.FLIGHT_DIR = Path("/nonexistent/path")
        try:
            result = _main_mod._start_camera("test_session")
        finally:
            _main_mod.FLIGHT_DIR = original_path
        self.assertIsNone(result)

    # ── _stop_camera ──────────────────────────────────────────────────────────

    def test_stop_camera_none_is_safe(self):
        """_stop_camera(None) must not raise."""
        try:
            _stop_camera(None)
        except Exception as exc:
            self.fail(f"_stop_camera(None) raised: {exc}")

    def test_stop_camera_calls_stop_method(self):
        mock_cam = _MockCameraCapture()
        mock_cam.started = True
        _stop_camera(mock_cam)
        self.assertTrue(mock_cam.stopped)

    def test_stop_camera_tolerates_exception(self):
        """If cam.stop() raises, _stop_camera must swallow the error."""
        bad_cam = MagicMock()
        bad_cam.stop.side_effect = RuntimeError("hardware fault")
        bad_cam._frame_index = 0
        bad_cam.output_dir   = Path("/tmp")
        try:
            _stop_camera(bad_cam)
        except RuntimeError:
            self.fail("_stop_camera did not swallow exception from cam.stop()")

    # ── _trigger_camera ───────────────────────────────────────────────────────

    def _make_node_mock(self, lat=18.215, lon=-67.147, alt=50.0):
        """Return a minimal MainNode mock with get_gps() wired."""
        node = MagicMock()
        node.get_gps.return_value = (lat, lon, alt)
        return node

    def test_trigger_none_cam_is_safe(self):
        """_trigger_camera(None, ...) must not raise."""
        node = self._make_node_mock()
        try:
            _trigger_camera(None, 0, (1.0, 2.0, 5.0), node)
        except Exception as exc:
            self.fail(f"_trigger_camera(None) raised: {exc}")

    def test_trigger_calls_cam_trigger(self):
        cam  = _MockCameraCapture()
        node = self._make_node_mock()
        _trigger_camera(cam, 3, (10.0, 20.0, 8.0), node)
        self.assertEqual(len(cam.triggers), 1)

    def test_trigger_context_waypoint_index(self):
        cam  = _MockCameraCapture()
        node = self._make_node_mock()
        _trigger_camera(cam, 7, (0.0, 0.0, 5.0), node)
        ctx = cam.triggers[0]
        self.assertEqual(ctx["waypoint_index"], 7)

    def test_trigger_context_enu_tuple(self):
        cam  = _MockCameraCapture()
        node = self._make_node_mock()
        _trigger_camera(cam, 0, (3.5, -1.2, 6.0), node)
        ctx = cam.triggers[0]
        self.assertEqual(ctx["enu"], (3.5, -1.2, 6.0))

    def test_trigger_context_gps_from_node(self):
        cam  = _MockCameraCapture()
        node = self._make_node_mock(lat=18.215, lon=-67.147, alt=52.0)
        _trigger_camera(cam, 0, (0.0, 0.0, 5.0), node)
        ctx = cam.triggers[0]
        self.assertAlmostEqual(ctx["gps"][0], 18.215, places=4)
        self.assertAlmostEqual(ctx["gps"][1], -67.147, places=4)
        self.assertAlmostEqual(ctx["gps"][2],  52.0,  places=2)

    def test_trigger_increments_frame_index(self):
        cam  = _MockCameraCapture()
        node = self._make_node_mock()
        _trigger_camera(cam, 0, (0.0, 0.0, 5.0), node)
        _trigger_camera(cam, 1, (1.0, 0.0, 5.0), node)
        self.assertEqual(cam._frame_index, 2)

    def test_trigger_missing_gps_does_not_raise(self):
        """If GPS is unavailable (None, None, None) trigger must still succeed."""
        cam  = _MockCameraCapture()
        node = self._make_node_mock(lat=None, lon=None, alt=None)
        try:
            _trigger_camera(cam, 0, (0.0, 0.0, 5.0), node)
        except Exception as exc:
            self.fail(f"_trigger_camera raised with no GPS: {exc}")

    # ── helper ────────────────────────────────────────────────────────────────

    def _start_with_mock_cls(self, mock_instance: _MockCameraCapture):
        """
        Call _start_camera with camera_capture module patched so that
        CameraCapture() returns mock_instance.
        """
        mock_module = MagicMock()
        mock_module.CameraCapture = MagicMock(return_value=mock_instance)
        with patch.dict(sys.modules, {"camera_capture": mock_module}):
            return _main_mod._start_camera("test_session")


# ══════════════════════════════════════════════════════════════════════════════
# 4. FlightMode enum
# ══════════════════════════════════════════════════════════════════════════════

class TestFlightMode(unittest.TestCase):
    """FlightMode enum values and membership."""

    def test_all_modes_exist(self):
        expected = {"IDLE", "DEBOUNCE", "NO_SCAN", "MANUAL", "AUTONOMOUS"}
        actual   = {m.name for m in FlightMode}
        self.assertEqual(actual, expected)

    def test_mode_values_are_strings(self):
        for mode in FlightMode:
            self.assertIsInstance(mode.value, str)

    def test_modes_are_unique(self):
        values = [m.value for m in FlightMode]
        self.assertEqual(len(values), len(set(values)))


# ══════════════════════════════════════════════════════════════════════════════
# 5. Preflight check individual guards (mocked MainNode)
# ══════════════════════════════════════════════════════════════════════════════

class TestPreflightCheckGuards(unittest.TestCase):
    """
    run_preflight_checks() individual check pass/fail paths.

    Each test constructs a MainNode mock that satisfies all checks
    except the one under test, then asserts the expected overall result.

    The SSD write test, launch file check, and waypoint check are file-system
    dependent; we patch Path.exists / Path.touch / Path.unlink as needed.
    """

    def _make_good_node(self):
        """Return a MainNode mock that satisfies every preflight check."""
        node = MagicMock()
        node.connected = True
        node.mode      = "STABILIZED"

        # GPS home
        home = MagicMock()
        home.geo.latitude  = 18.215
        home.geo.longitude = -67.147
        home.geo.altitude  = 10.0
        node.get_home.return_value = home

        # EKF: get_pos() returns stable Z across 20+ calls
        node.get_pos.return_value = (0.0, 0.0, 1.500)

        # SLAM vision
        node.vision_pose_received = True

        # Waypoints
        wp = MagicMock()
        wp.command = 16
        node.get_nav_waypoints.return_value = [wp]

        return node

    def _run(self, node):
        """
        Run preflight checks with filesystem-dependent paths patched so
        the test is hermetic — no actual files created or required.
        """
        with patch.object(_main_mod.Path, "exists",   return_value=True), \
             patch.object(_main_mod.Path, "touch",    return_value=None), \
             patch.object(_main_mod.Path, "unlink",   return_value=None):
            return _main_mod.run_preflight_checks(node)

    def test_all_checks_pass_with_good_node(self):
        node = self._make_good_node()
        result = self._run(node)
        self.assertTrue(result)

    def test_fcu_not_connected_fails(self):
        node = self._make_good_node()
        node.connected = False
        result = self._run(node)
        self.assertFalse(result)

    def test_ekf_unstable_fails(self):
        """
        Simulate EKF instability by making get_pos() return a different Z
        on every call — stable counter never reaches EKF_STABLE_COUNT.
        """
        node = self._make_good_node()
        counter = [0]

        def _unstable_pos():
            counter[0] += 1
            return (0.0, 0.0, float(counter[0]) * 0.5)

        node.get_pos.side_effect = _unstable_pos
        result = self._run(node)
        self.assertFalse(result)

    def test_no_waypoints_fails(self):
        node = self._make_good_node()
        node.get_nav_waypoints.return_value = []
        result = self._run(node)
        self.assertFalse(result)

    def test_slam_bridge_not_publishing_fails(self):
        node = self._make_good_node()
        node.vision_pose_received = False
        result = self._run(node)
        self.assertFalse(result)

    def test_gps_unavailable_is_only_warn(self):
        """
        GPS unavailable is a warning in preflight (SLAM is primary source).
        All other checks passing → overall result must still be True.
        """
        node = self._make_good_node()
        node.get_home.return_value = None   # GPS not locked
        result = self._run(node)
        self.assertTrue(result)


# ── Test runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 65)
    print("  DronePi Main Orchestrator — Integration Test Suite")
    print(f"  Module : {_MAIN_PATH}")
    print("=" * 65)
    unittest.main(verbosity=2)
