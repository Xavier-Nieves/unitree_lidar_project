#!/usr/bin/env python3
"""tests/test_hailo_flight.py — Standalone Hailo integration test.

Tests HailoDevice, HailoOpticalFlow, and HailoGroundClassifier independently
before integrating with the flight stack. Run this FIRST to confirm the Hailo
chain is working on your hardware before attempting a flight.

NOT a production module — does not import from flight/ or watchdog_core/.
Run entirely in hailo_inference_env.

Usage:
    source ~/hailo_inference_env/bin/activate
    python3 tests/test_hailo_flight.py

    # Test individual components:
    python3 tests/test_hailo_flight.py --test device
    python3 tests/test_hailo_flight.py --test flow
    python3 tests/test_hailo_flight.py --test ground
    python3 tests/test_hailo_flight.py --test all        # default

    # Use a static image instead of live camera:
    python3 tests/test_hailo_flight.py --image /path/to/frame.jpg

    # Run N inference passes and report average latency:
    python3 tests/test_hailo_flight.py --passes 100

Expected results on RPi 5 + Hailo-8 HAT+ (26 TOPS):
    Device open:       < 2s
    Model load:        < 5s per model
    Flow inference:    < 30ms per frame pair
    Ground inference:  < 25ms per frame
    Flow publish rate: > 20 Hz sustained
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent directory so hailo/ modules are importable
sys.path.insert(0, str(Path(__file__).parent.parent))

HAILO_RESOURCES = Path("/usr/local/hailo/resources/models/hailo8")
FLOW_HEF        = HAILO_RESOURCES / "scdepthv3.hef"
CLASS_HEF       = HAILO_RESOURCES / "yolov8m.hef"
DEVICE_NODE     = Path("/dev/hailo0")


def _header(title: str) -> None:
    print(f"\n{'='*55}")
    print(f"  {title}")
    print(f"{'='*55}")


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _info(msg: str) -> None:
    print(f"  [INFO] {msg}")


# ── Test 1: Device ────────────────────────────────────────────────────────────

def test_device() -> bool:
    _header("TEST 1 — HailoDevice")
    from hailo.hailo_device import HailoDevice

    device = HailoDevice()

    # 1a. OS-level availability
    if device.is_available():
        _pass(f"/dev/hailo0 exists and is accessible")
    else:
        _fail("/dev/hailo0 not found — check PCIe connection and driver")
        _info("Run: sudo dmesg | grep hailo")
        return False

    # 1b. VDevice open
    t0 = time.monotonic()
    try:
        device.open()
        elapsed = (time.monotonic() - t0) * 1000
        _pass(f"VDevice opened in {elapsed:.0f}ms")
    except RuntimeError as exc:
        _fail(f"VDevice open failed: {exc}")
        return False

    # 1c. Temperature
    temp = device.get_temperature()
    if temp > 0:
        _pass(f"Die temperature: {temp:.1f}°C")
        if temp > 80:
            _info("Temperature above 80°C — check Hailo cooling block is fitted")
    else:
        _info("Temperature unavailable (hailortcli not in PATH or no response)")

    # 1d. Model loading — both models
    for name, path in [("scdepthv3", FLOW_HEF), ("yolov8m", CLASS_HEF)]:
        if not path.exists():
            _info(f"{path.name} not found — skipping load test for {name}")
            continue
        try:
            t0 = time.monotonic()
            device.load_model(str(path))
            elapsed = (time.monotonic() - t0) * 1000
            _pass(f"{name}.hef loaded in {elapsed:.0f}ms")
        except Exception as exc:
            _fail(f"{name}.hef load failed: {exc}")

    device.shutdown()
    _pass("VDevice released cleanly")
    return True


# ── Test 2: Optical Flow ──────────────────────────────────────────────────────

def test_optical_flow(image_path: str = None, passes: int = 10) -> bool:
    _header("TEST 2 — HailoOpticalFlow")
    import numpy as np

    if not FLOW_HEF.exists():
        _fail(f"Flow HEF not found: {FLOW_HEF}")
        _info("Set HAILO_FLOW_HEF env var to point to a valid optical flow .hef")
        return False

    from hailo.hailo_device    import HailoDevice
    from hailo.hailo_optical_flow import HailoOpticalFlow

    with HailoDevice() as device:
        try:
            network = device.load_model(str(FLOW_HEF))
        except Exception as exc:
            _fail(f"Model load failed: {exc}")
            return False

        flow = HailoOpticalFlow(network_group=network, fps=30.0, agl_default=5.0)

        # Generate or load test frames
        if image_path and Path(image_path).exists():
            import cv2
            img    = cv2.imread(image_path)
            frame1 = img
            frame2 = img.copy()
            # Simulate small motion by shifting frame2 by 10px
            M      = np.float32([[1, 0, 10], [0, 1, 5]])
            frame2 = cv2.warpAffine(frame2, M, (frame2.shape[1], frame2.shape[0]))
            _info(f"Using image: {image_path} (shifted 10px to simulate motion)")
        else:
            _info("No image provided — using synthetic noise frames")
            frame1 = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            frame2 = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # First push — seeds the previous frame buffer, returns invalid result
        result = flow.push_frame(frame1, agl=5.0)
        if not result["valid"]:
            _pass("First frame correctly returns valid=False (no frame pair yet)")
        else:
            _fail("First frame should return valid=False — check push_frame() logic")

        # Inference passes
        latencies = []
        for i in range(passes):
            result = flow.push_frame(frame2, agl=5.0)
            latencies.append(result["latency_ms"])

        avg_lat = sum(latencies) / len(latencies)
        max_lat = max(latencies)

        _pass(f"{passes} inference passes completed")
        _pass(f"Average latency: {avg_lat:.1f}ms  Max: {max_lat:.1f}ms")

        if avg_lat < 50:
            _pass(f"Latency within target (< 50ms)")
        else:
            _info(f"Latency above 50ms target — may affect EKF2 fusion quality")

        last = flow.push_frame(frame2, agl=5.0)
        _info(f"Last result: vx={last['vx']:.3f}m/s  vy={last['vy']:.3f}m/s  "
              f"conf={last['confidence']:.3f}  valid={last['valid']}")

    return True


# ── Test 3: Ground Classifier ─────────────────────────────────────────────────

def test_ground_classifier(image_path: str = None, passes: int = 5) -> bool:
    _header("TEST 3 — HailoGroundClassifier")
    import numpy as np

    if not CLASS_HEF.exists():
        _fail(f"Class HEF not found: {CLASS_HEF}")
        return False

    from hailo.hailo_device      import HailoDevice
    from hailo.hailo_ground_class import HailoGroundClassifier

    with HailoDevice() as device:
        try:
            network = device.load_model(str(CLASS_HEF))
        except Exception as exc:
            _fail(f"Model load failed: {exc}")
            return False

        classifier = HailoGroundClassifier(network_group=network)

        if image_path and Path(image_path).exists():
            import cv2
            frame = cv2.imread(image_path)
            _info(f"Using image: {image_path}")
        else:
            _info("No image provided — using synthetic green frame (grass proxy)")
            frame = np.zeros((640, 640, 3), dtype=np.uint8)
            frame[:, :, 1] = 120   # green channel — grass-like

        latencies = []
        for i in range(passes):
            result = classifier.classify(frame)
            latencies.append(result["latency_ms"])

        avg_lat = sum(latencies) / len(latencies)
        _pass(f"{passes} inference passes completed")
        _pass(f"Average latency: {avg_lat:.1f}ms")

        last = classifier.classify(frame)
        _info(f"Classification: label={last['label']}  "
              f"confidence={last['confidence']:.3f}  "
              f"detections={last['detections'][:3]}")

        if avg_lat < 50:
            _pass("Latency within target (< 50ms)")
        else:
            _info("Latency above 50ms — ground class runs every 30 frames, acceptable")

    return True


# ── Test 4: Live camera flow (optional) ───────────────────────────────────────

def test_live_camera(duration_s: float = 5.0) -> bool:
    _header("TEST 4 — Live Camera + Optical Flow (5 seconds)")
    import cv2
    import numpy as np

    if not FLOW_HEF.exists():
        _info("Flow HEF not found — skipping live camera test")
        return True

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        _info("No camera on /dev/video0 — skipping live camera test")
        return True

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    from hailo.hailo_device       import HailoDevice
    from hailo.hailo_optical_flow import HailoOpticalFlow

    frame_count = 0
    with HailoDevice() as device:
        network = device.load_model(str(FLOW_HEF))
        flow    = HailoOpticalFlow(network_group=network, fps=30.0)

        t_end = time.monotonic() + duration_s
        while time.monotonic() < t_end:
            ret, frame = cap.read()
            if not ret:
                break
            result = flow.push_frame(frame, agl=5.0)
            if result["valid"]:
                frame_count += 1
                print(
                    f"\r  vx={result['vx']:+.3f}  vy={result['vy']:+.3f}  "
                    f"conf={result['confidence']:.2f}  "
                    f"lat={result['latency_ms']:.1f}ms  "
                    f"fps={flow.get_fps():.1f}",
                    end="", flush=True,
                )

    cap.release()
    print()
    effective_fps = frame_count / duration_s
    _pass(f"Live camera test: {frame_count} frames in {duration_s:.0f}s "
          f"({effective_fps:.1f} Hz effective)")
    if effective_fps >= 20:
        _pass("Frame rate meets 20 Hz target for EKF2 fusion")
    else:
        _info("Frame rate below 20 Hz — check camera and Hailo load")
    return True


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Standalone Hailo in-flight integration test"
    )
    parser.add_argument(
        "--test",
        choices=["device", "flow", "ground", "camera", "all"],
        default="all",
        help="Which test to run (default: all)",
    )
    parser.add_argument(
        "--image",
        default=None,
        help="Path to a test image file (used instead of synthetic frames)",
    )
    parser.add_argument(
        "--passes",
        type=int,
        default=10,
        help="Number of inference passes per test (default: 10)",
    )
    args = parser.parse_args()

    results = {}

    if args.test in ("device", "all"):
        results["device"] = test_device()

    if args.test in ("flow", "all"):
        results["flow"] = test_optical_flow(
            image_path=args.image, passes=args.passes
        )

    if args.test in ("ground", "all"):
        results["ground"] = test_ground_classifier(
            image_path=args.image, passes=args.passes
        )

    if args.test in ("camera", "all"):
        results["camera"] = test_live_camera()

    _header("SUMMARY")
    all_passed = True
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")
        if not passed:
            all_passed = False

    if all_passed:
        print("\n  All tests passed — Hailo flight integration ready")
        print("  Next step: start hailo_flight_node.py and check ROS 2 topics")
        print("    ~/hailo_inference_env/bin/python3 hailo/hailo_flight_node.py &")
        print("    ros2 topic hz /hailo/optical_flow")
        print("    ros2 topic hz /hailo/ground_class")
    else:
        print("\n  One or more tests failed — resolve before flight")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
