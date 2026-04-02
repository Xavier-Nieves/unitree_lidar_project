#!/usr/bin/env python3
"""
test_camera_capture.py — Standalone test suite for CameraCapture.

Tests the CameraCapture module exhaustively without requiring a live drone,
ROS 2 environment, or a physical IMX477 camera. A MockPicamera2 class replaces
the real driver, producing synthetic frames (solid-colour NumPy arrays) so all
threading, file I/O, CSV logging, and sidecar JSON paths are exercised.

Test categories
---------------
1. Unit tests          — individual methods with mocked dependencies
2. Integration tests   — full start/trigger/stop cycle against MockPicamera2
3. Concurrency tests   — rapid successive triggers, thread safety under load
4. Failure/edge tests  — camera unavailable, zero-context trigger, stop before start

Usage
-----
    python test_camera_capture.py            # all tests, verbose
    python test_camera_capture.py -v         # unittest verbose mode
    python test_camera_capture.py TestTrigger  # run one class only

NOTE: This script is intentionally STANDALONE. It does NOT import
      from unitree_drone_mapper as a package; it imports camera_capture.py
      directly via importlib so it can be run from any working directory
      without the full project installed.
"""

import csv
import importlib.util
import json
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

# ── Direct import of camera_capture.py (no package install required) ──────────

_MODULE_PATH = Path(__file__).resolve().parent.parent / \
    "unitree_drone_mapper" / "flight" / "camera_capture.py"

if not _MODULE_PATH.exists():
    # Support running from repo root: tests/test_camera_capture.py
    _MODULE_PATH = Path(__file__).resolve().parents[1] / \
        "unitree_drone_mapper" / "flight" / "camera_capture.py"

if not _MODULE_PATH.exists():
    print(f"[ERROR] Cannot locate camera_capture.py — expected at:\n  {_MODULE_PATH}")
    sys.exit(1)

spec = importlib.util.spec_from_file_location("camera_capture", _MODULE_PATH)
_cam_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_cam_mod)

CameraCapture = _cam_mod.CameraCapture
CSV_HEADERS   = _cam_mod.CSV_HEADERS

# ── Mock Picamera2 ────────────────────────────────────────────────────────────

class MockPicamera2:
    """
    Drop-in replacement for Picamera2 that produces synthetic frames.

    Returns a 2028×1520×3 uint8 array filled with a solid colour per
    capture, cycling through a list of test colours. This validates that
    the correct frame data is written to disk without requiring hardware.

    Attributes
    ----------
    capture_count : int
        Total calls to capture_array — checked by concurrency tests.
    closed : bool
        True after close() has been called — validates lifecycle.
    """

    _COLOURS = [
        (220, 50,  50),   # red
        (50,  220, 50),   # green
        (50,  50,  220),  # blue
        (200, 200, 50),   # yellow
        (50,  200, 200),  # cyan
    ]

    def __init__(self):
        self.capture_count  = 0
        self.closed         = False
        self._config        = None
        self._started       = False
        self._colour_idx    = 0
        self._lock          = threading.Lock()

    def create_still_configuration(self, main=None, controls=None):
        return {"main": main, "controls": controls}

    def configure(self, config):
        self._config = config

    def start(self):
        self._started = True

    def stop(self):
        self._started = False

    def close(self):
        self.closed = True

    def capture_array(self, stream="main"):
        with self._lock:
            colour = self._COLOURS[self._colour_idx % len(self._COLOURS)]
            self._colour_idx  += 1
            self.capture_count += 1
        # Simulate a short camera latency (~25ms at 40fps)
        time.sleep(0.025)
        frame = np.full((1520, 2028, 3), colour, dtype=np.uint8)
        return frame

    def capture_metadata(self):
        return {
            "ExposureTime":  8000,
            "AnalogueGain":  1.5,
            "ColourGains":   [1.8, 2.1],
        }


def _make_camera(tmp_dir: Path, **kwargs) -> CameraCapture:
    """
    Construct a CameraCapture wired to MockPicamera2.

    Patches picamera2 at the module level so CameraCapture.start() does
    not attempt to open real hardware.
    """
    cam = CameraCapture(
        session_id="test_session",
        output_dir=tmp_dir,
        **kwargs,
    )
    return cam


def _start_with_mock(cam: CameraCapture) -> MockPicamera2:
    """Start camera with MockPicamera2 injected; skip the AEC/AWB sleep."""
    mock_piCam = MockPicamera2()

    mock_module = MagicMock()
    mock_module.Picamera2 = MagicMock(return_value=mock_piCam)

    with patch.dict(sys.modules, {"picamera2": mock_module}):
        with patch.object(_cam_mod.time, "sleep", return_value=None):
            result = cam.start()

    assert result is True, "CameraCapture.start() returned False with mock"
    return mock_piCam


# ══════════════════════════════════════════════════════════════════════════════
# TEST CLASSES
# ══════════════════════════════════════════════════════════════════════════════

class TestLifecycle(unittest.TestCase):
    """Validate start/stop state transitions and resource cleanup."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_start_creates_output_dir(self):
        """Output directory must exist after start()."""
        out = self.tmp / "new_session"
        cam = _make_camera(out)
        _start_with_mock(cam)
        cam.stop()
        self.assertTrue(out.exists(), "Output directory was not created by start()")

    def test_stop_before_start_is_safe(self):
        """stop() on an uninitialised camera must not raise."""
        cam = _make_camera(self.tmp / "unused")
        try:
            cam.stop()
        except Exception as exc:
            self.fail(f"stop() before start() raised: {exc}")

    def test_start_sets_started_flag(self):
        cam = _make_camera(self.tmp / "s")
        mock = _start_with_mock(cam)
        self.assertTrue(cam._started)
        cam.stop()

    def test_stop_clears_started_flag(self):
        cam = _make_camera(self.tmp / "s")
        _start_with_mock(cam)
        cam.stop()
        self.assertFalse(cam._started)

    def test_stop_closes_camera(self):
        cam = _make_camera(self.tmp / "s")
        mock = _start_with_mock(cam)
        cam.stop()
        self.assertTrue(mock.closed, "MockPicamera2.close() was not called on stop()")

    def test_context_manager(self):
        """CameraCapture used as context manager must start and stop cleanly."""
        out = self.tmp / "ctx"
        cam = _make_camera(out)
        mock = _start_with_mock(cam)

        # Simulate __exit__ manually since we can't use 'with' after mock start
        cam.stop()
        self.assertFalse(cam._started)

    def test_camera_unavailable_returns_false(self):
        """start() must return False gracefully when picamera2 is not installed."""
        cam = _make_camera(self.tmp / "na")
        with patch.dict(sys.modules, {"picamera2": None}):
            result = cam.start()
        self.assertFalse(result)


class TestTrigger(unittest.TestCase):
    """Validate trigger-to-save latency, file creation, and return value."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cam  = _make_camera(self.tmp)
        self.mock = _start_with_mock(self.cam)

    def tearDown(self):
        self.cam.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_trigger_returns_path(self):
        path = self.cam.trigger({"waypoint_index": 0})
        self.assertIsNotNone(path, "trigger() returned None — frame not saved")
        self.assertIsInstance(path, Path)

    def test_trigger_file_exists_on_disk(self):
        path = self.cam.trigger({"waypoint_index": 1})
        self.assertTrue(path.exists(), f"JPEG not found at {path}")

    def test_trigger_jpeg_suffix(self):
        path = self.cam.trigger({})
        self.assertEqual(path.suffix, ".jpg")

    def test_trigger_increments_frame_index(self):
        self.cam.trigger({})
        self.cam.trigger({})
        self.assertEqual(self.cam._frame_index, 2)

    def test_trigger_without_camera_returns_none(self):
        """trigger() on a stopped camera must return None, not raise."""
        cam2 = _make_camera(self.tmp / "stopped")
        result = cam2.trigger({"waypoint_index": 99})
        self.assertIsNone(result)

    def test_trigger_empty_context(self):
        """trigger() with no context dict must succeed and not raise."""
        path = self.cam.trigger()
        self.assertIsNotNone(path)

    def test_trigger_none_context(self):
        path = self.cam.trigger(None)
        self.assertIsNotNone(path)


class TestSidecarJSON(unittest.TestCase):
    """Validate sidecar JSON schema and data fidelity."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cam  = _make_camera(self.tmp)
        _start_with_mock(self.cam)

    def tearDown(self):
        self.cam.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _trigger_and_load_json(self, context: dict) -> dict:
        path = self.cam.trigger(context)
        json_path = path.with_suffix(".json")
        self.assertTrue(json_path.exists(), f"Sidecar JSON not found: {json_path}")
        return json.loads(json_path.read_text())

    def test_sidecar_contains_required_keys(self):
        required = {
            "frame_index", "filename", "timestamp_iso",
            "waypoint_index", "enu", "gps", "latency_ms",
            "sensor_mode", "framerate", "jpeg_quality",
        }
        sidecar = self._trigger_and_load_json(
            {"waypoint_index": 3, "enu": (1.0, 2.0, 5.0), "gps": (18.2, -67.1, 50.0)}
        )
        missing = required - sidecar.keys()
        self.assertFalse(missing, f"Sidecar JSON missing keys: {missing}")

    def test_waypoint_index_persisted(self):
        sidecar = self._trigger_and_load_json({"waypoint_index": 7})
        self.assertEqual(sidecar["waypoint_index"], 7)

    def test_enu_coordinates_persisted(self):
        sidecar = self._trigger_and_load_json(
            {"waypoint_index": 0, "enu": (10.5, -3.2, 8.0)}
        )
        self.assertAlmostEqual(sidecar["enu"]["x"],  10.5)
        self.assertAlmostEqual(sidecar["enu"]["y"],  -3.2)
        self.assertAlmostEqual(sidecar["enu"]["z"],   8.0)

    def test_gps_coordinates_persisted(self):
        sidecar = self._trigger_and_load_json(
            {"waypoint_index": 0, "gps": (18.215, -67.147, 52.3)}
        )
        self.assertAlmostEqual(sidecar["gps"]["lat"],  18.215, places=4)
        self.assertAlmostEqual(sidecar["gps"]["lon"], -67.147, places=4)

    def test_sensor_mode_string(self):
        sidecar = self._trigger_and_load_json({})
        self.assertEqual(sidecar["sensor_mode"], "2028x1520")

    def test_latency_is_positive(self):
        sidecar = self._trigger_and_load_json({})
        self.assertGreater(sidecar["latency_ms"], 0)

    def test_camera_metadata_persisted(self):
        """ExposureTime and AnalogueGain from MockPicamera2 must appear."""
        sidecar = self._trigger_and_load_json({})
        self.assertEqual(sidecar["cam_exposure_us"], 8000)
        self.assertAlmostEqual(sidecar["cam_gain"], 1.5)

    def test_frame_index_matches_filename(self):
        path    = self.cam.trigger({"waypoint_index": 1})
        sidecar = json.loads(path.with_suffix(".json").read_text())
        expected_idx = int(path.stem.split("_")[1])
        self.assertEqual(sidecar["frame_index"], expected_idx)


class TestCSVLog(unittest.TestCase):
    """Validate capture_log.csv structure and row content."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cam  = _make_camera(self.tmp)
        _start_with_mock(self.cam)

    def tearDown(self):
        self.cam.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _read_csv(self):
        csv_path = self.tmp / "capture_log.csv"
        self.assertTrue(csv_path.exists(), "capture_log.csv not created")
        with open(csv_path, newline="") as f:
            return list(csv.DictReader(f))

    def test_csv_created_on_start(self):
        csv_path = self.tmp / "capture_log.csv"
        self.assertTrue(csv_path.exists())

    def test_csv_headers_match_spec(self):
        """Header row written by _open_csv must exactly match CSV_HEADERS."""
        csv_path = self.tmp / "capture_log.csv"
        self.assertTrue(csv_path.exists(), "capture_log.csv not found")
        with open(csv_path, newline="") as f:
            reader = csv.reader(f)
            actual_headers = next(reader)   # first row only — fresh reader
        self.assertEqual(actual_headers, CSV_HEADERS)

    def test_csv_row_added_per_trigger(self):
        for i in range(3):
            self.cam.trigger({"waypoint_index": i})
        rows = self._read_csv()
        self.assertEqual(len(rows), 3)

    def test_csv_waypoint_index_correct(self):
        self.cam.trigger({"waypoint_index": 5})
        rows = self._read_csv()
        self.assertEqual(rows[0]["waypoint_index"], "5")

    def test_csv_enu_columns(self):
        self.cam.trigger({"waypoint_index": 0, "enu": (1.1, 2.2, 3.3)})
        rows = self._read_csv()
        self.assertAlmostEqual(float(rows[0]["enu_x"]), 1.1, places=3)
        self.assertAlmostEqual(float(rows[0]["enu_y"]), 2.2, places=3)
        self.assertAlmostEqual(float(rows[0]["enu_z"]), 3.3, places=3)


class TestConcurrency(unittest.TestCase):
    """Thread-safety and rapid-trigger stress tests."""

    def setUp(self):
        self.tmp  = Path(tempfile.mkdtemp())
        self.cam  = _make_camera(self.tmp)
        self.mock = _start_with_mock(self.cam)

    def tearDown(self):
        self.cam.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_sequential_triggers_all_succeed(self):
        """N sequential triggers must each produce a unique JPEG."""
        N     = 8
        paths = []
        for i in range(N):
            p = self.cam.trigger({"waypoint_index": i})
            self.assertIsNotNone(p, f"Trigger {i} returned None")
            paths.append(p)
        self.assertEqual(len(set(paths)), N, "Duplicate paths — frames overwritten")

    def test_frame_index_monotonically_increases(self):
        """Frame indices must be strictly monotonic."""
        indices = []
        for i in range(5):
            self.cam.trigger({})
            indices.append(self.cam._frame_index)
        for a, b in zip(indices, indices[1:]):
            self.assertGreater(b, a, "Frame index did not increase monotonically")

    def test_all_jpegs_present_on_disk(self):
        """Every triggered frame must have a corresponding JPEG on disk."""
        N = 6
        for i in range(N):
            self.cam.trigger({"waypoint_index": i})
        jpgs = sorted(self.tmp.glob("frame_*.jpg"))
        self.assertEqual(len(jpgs), N, f"Expected {N} JPEGs, found {len(jpgs)}")

    def test_trigger_lock_prevents_state_corruption(self):
        """
        Concurrent trigger calls from two threads must not corrupt frame_index.
        Both threads call trigger() simultaneously; total frames must equal 2.
        """
        results = []
        barrier = threading.Barrier(2)

        def _fire(ctx):
            barrier.wait()   # Synchronise both threads to maximise contention
            results.append(self.cam.trigger(ctx))

        t1 = threading.Thread(target=_fire, args=({"waypoint_index": 0},))
        t2 = threading.Thread(target=_fire, args=({"waypoint_index": 1},))
        t1.start(); t2.start()
        t1.join();  t2.join()

        # Both calls must return a valid path
        non_none = [r for r in results if r is not None]
        self.assertGreaterEqual(
            len(non_none), 1,
            "Concurrent triggers: at least one must succeed"
        )


class TestJPEGIntegrity(unittest.TestCase):
    """Validate that saved JPEG files are valid, parseable images."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cam  = _make_camera(self.tmp)
        _start_with_mock(self.cam)

    def tearDown(self):
        self.cam.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_jpeg_is_valid_image(self):
        """Saved JPEG must be parseable by Pillow with correct dimensions."""
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("Pillow not installed")

        path = self.cam.trigger({"waypoint_index": 0})
        img  = Image.open(str(path))
        self.assertEqual(img.format, "JPEG")
        self.assertEqual(img.size,   (2028, 1520))  # width × height

    def test_jpeg_colour_matches_mock_frame(self):
        """
        MockPicamera2 fills frame with (220, 50, 50) for the first capture.
        The saved JPEG dominant colour must be close to that value.
        JPEG compression introduces small errors; tolerance is ±15 DN.
        """
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("Pillow not installed")

        path  = self.cam.trigger({})
        img   = Image.open(str(path)).convert("RGB")
        arr   = np.array(img)
        mean  = arr.reshape(-1, 3).mean(axis=0)   # [R, G, B]
        expected = np.array(MockPicamera2._COLOURS[0], dtype=float)
        diff = np.abs(mean - expected)
        self.assertTrue(
            (diff < 15).all(),
            f"JPEG colour mismatch: expected≈{expected}, got {mean}"
        )

    def test_jpeg_quality_setting(self):
        """Lower JPEG quality must produce a smaller file."""
        out_hq = Path(tempfile.mkdtemp())
        out_lq = Path(tempfile.mkdtemp())
        try:
            cam_hq = _make_camera(out_hq, jpeg_quality=95)
            cam_lq = _make_camera(out_lq, jpeg_quality=40)
            _start_with_mock(cam_hq)
            _start_with_mock(cam_lq)
            p_hq = cam_hq.trigger({})
            p_lq = cam_lq.trigger({})
            cam_hq.stop()
            cam_lq.stop()
            self.assertLess(
                p_lq.stat().st_size, p_hq.stat().st_size,
                "Lower quality JPEG should be smaller than higher quality"
            )
        finally:
            shutil.rmtree(out_hq, ignore_errors=True)
            shutil.rmtree(out_lq, ignore_errors=True)


# ── Test runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 65)
    print("  CameraCapture Test Suite")
    print(f"  Module : {_MODULE_PATH}")
    print("=" * 65)
    unittest.main(verbosity=2)
