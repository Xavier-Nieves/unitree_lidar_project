"""hailo/hailo_optical_flow.py — Nadir optical flow velocity estimator.

Runs a compiled optical flow .hef model on the Hailo-8 and outputs body-frame
XY velocity estimates (vx, vy) from consecutive nadir camera frames.

The output feeds /hailo/optical_flow (geometry_msgs/TwistStamped) via the
HailoFlightNode ROS 2 publisher. FlowBridge then forwards the velocity to
/mavros/odometry/out for EKF2 fusion via EKF2_EV_CTRL bit 2.

Environment: hailo_inference_env

Model options (in preference order):
    1. A custom RAFT-small .hef compiled with the Hailo Dataflow Compiler
       (best accuracy — requires offline compilation step)
    2. SCDepthV3 .hef already on your system at:
       /usr/local/hailo/resources/models/hailo8/scdepthv3.hef
       (depth per pixel → gradient → approximate flow — fallback mode)

In both cases the output is normalised to a velocity estimate in metres/second
using the known altitude (AGL) and camera intrinsics to convert pixel
displacement to metric velocity.

Pixel displacement → metric velocity conversion:
    Given:
        px_disp   = pixel displacement between frames (from flow output)
        fps       = camera frame rate
        agl       = altitude above ground in metres
        focal_px  = focal length in pixels (from camera intrinsics)

    metric_velocity = (px_disp / fps) * (agl / focal_px)

    This is the standard pinhole camera projection — identical to what
    PX4's own px4flow sensor does internally.

Design notes:
    - Frame pairs are maintained internally. Caller only pushes single frames.
    - Confidence is estimated from the flow magnitude consistency across the
      output tensor — high variance = low confidence (turbulent scene).
    - All numpy operations use float32 to satisfy hailort 4.23.0's numpy < 2
      constraint (np.float32 arrays are safe regardless of numpy version).
"""

import time
import numpy as np


# Default camera intrinsics for IMX477 with Tamron 4-12mm at mid-zoom (~8mm)
# Update these with your actual calibrated values from the camera calibration step.
# focal_px = (focal_length_mm / sensor_pixel_size_um) * 1000
# IMX477 pixel pitch = 1.55 µm, focal = 8mm → focal_px ≈ 5161
DEFAULT_FOCAL_PX  = 5161.0
DEFAULT_INPUT_H   = 480
DEFAULT_INPUT_W   = 640
DEFAULT_FPS       = 30.0

# Minimum flow vector magnitude to count as valid (filters static noise)
MIN_FLOW_MAGNITUDE = 0.5


class HailoOpticalFlow:
    """Estimates body-frame XY velocity from consecutive nadir camera frames.

    Args:
        network_group: Configured network group from HailoDevice.load_model().
        input_h:       Model input height in pixels. Default 480.
        input_w:       Model input width in pixels. Default 640.
        fps:           Camera frame rate used for velocity scaling. Default 30.
        focal_px:      Focal length in pixels from camera calibration.
        agl_default:   Default altitude AGL in metres if not supplied per-call.
    """

    def __init__(
        self,
        network_group,
        input_h:     int   = DEFAULT_INPUT_H,
        input_w:     int   = DEFAULT_INPUT_W,
        fps:         float = DEFAULT_FPS,
        focal_px:    float = DEFAULT_FOCAL_PX,
        agl_default: float = 5.0,
    ):
        self._network_group = network_group
        self._input_h       = input_h
        self._input_w       = input_w
        self._fps           = fps
        self._focal_px      = focal_px
        self._agl_default   = agl_default

        self._prev_frame    = None   # Preprocessed frame from t-1
        self._latency_buf   = []     # Rolling latency measurements (ms)
        self._fps_buf       = []     # Rolling FPS measurements
        self._last_call_t   = None

        # Resolve vstream params once at construction
        try:
            from hailo_platform import (
                InputVStreamParams, OutputVStreamParams,
                FormatType, HailoStreamInterface
            )
            self._input_params  = InputVStreamParams.make_from_network_group(
                network_group, quantized=False, format_type=FormatType.UINT8
            )
            self._output_params = OutputVStreamParams.make_from_network_group(
                network_group, quantized=False, format_type=FormatType.FLOAT32
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to create vstream params: {exc}") from exc

    # ── Public API ────────────────────────────────────────────────────────────

    def push_frame(self, frame: np.ndarray, agl: float = None) -> dict:
        """Push a new nadir frame and return a velocity estimate.

        On the first call, stores the frame and returns a zero result —
        a frame pair is required for flow estimation.

        Args:
            frame: BGR or greyscale numpy array from camera. Any resolution —
                   resized internally to (input_h, input_w).
            agl:   Altitude above ground in metres. If None, uses agl_default.
                   More accurate AGL (from LiDAR or baro) gives more accurate
                   metric velocity — pass node.get_pos()[2] from MainNode.

        Returns:
            dict with keys:
                vx         (float) — forward velocity m/s (body frame)
                vy         (float) — lateral velocity m/s (body frame)
                confidence (float) — 0.0–1.0, reliability estimate
                latency_ms (float) — inference wall time in milliseconds
                valid      (bool)  — False on first call or inference failure
        """
        t0  = time.monotonic()
        agl = agl if agl is not None else self._agl_default

        current = self._preprocess(frame)

        if self._prev_frame is None:
            self._prev_frame = current
            return self._zero_result(latency_ms=0.0)

        result = self._infer(self._prev_frame, current, agl, t0)
        self._prev_frame = current
        self._update_fps()
        return result

    def get_fps(self) -> float:
        """Return rolling average inference rate in Hz."""
        if len(self._fps_buf) < 2:
            return 0.0
        intervals = [self._fps_buf[i+1] - self._fps_buf[i]
                     for i in range(len(self._fps_buf) - 1)]
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0.0

    def get_avg_latency_ms(self) -> float:
        """Return rolling average inference latency in milliseconds."""
        if not self._latency_buf:
            return 0.0
        return sum(self._latency_buf) / len(self._latency_buf)

    def reset(self) -> None:
        """Clear the stored previous frame. Call on mode transitions."""
        self._prev_frame = None

    # ── Private ───────────────────────────────────────────────────────────────

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """Resize, convert to greyscale, and format as (1, H, W, 1) uint8."""
        import cv2
        resized = cv2.resize(frame, (self._input_w, self._input_h))
        if len(resized.shape) == 3:
            grey = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        else:
            grey = resized
        # Shape: (1, H, W, 1) — batch=1, single channel
        return np.expand_dims(np.expand_dims(grey, axis=0), axis=-1).astype(np.uint8)

    def _infer(
        self,
        frame_t0: np.ndarray,
        frame_t1: np.ndarray,
        agl:      float,
        t_start:  float,
    ) -> dict:
        """Run inference on a frame pair and convert output to metric velocity."""
        try:
            from hailo_platform import InferVStreams

            # Concatenate frames along channel axis: (1, H, W, 2)
            # This is the standard two-frame input format for flow networks
            input_tensor = np.concatenate([frame_t0, frame_t1], axis=-1)

            # Get input stream name from params
            input_name = list(self._input_params)[0].name

            with InferVStreams(
                self._network_group,
                self._input_params,
                self._output_params,
            ) as pipeline:
                input_dict = {input_name: input_tensor}
                raw_outputs = pipeline.infer(input_dict)

            latency_ms = (time.monotonic() - t_start) * 1000.0
            self._latency_buf.append(latency_ms)
            if len(self._latency_buf) > 30:
                self._latency_buf.pop(0)

            return self._parse_flow_output(raw_outputs, agl, latency_ms)

        except Exception as exc:
            latency_ms = (time.monotonic() - t_start) * 1000.0
            print(f"[HailoOpticalFlow] Inference error: {exc}")
            return self._zero_result(latency_ms=latency_ms)

    def _parse_flow_output(
        self, raw_outputs: dict, agl: float, latency_ms: float
    ) -> dict:
        """Convert raw flow tensor to metric velocity dict.

        The flow output tensor shape depends on the compiled model:
          - RAFT-style: (1, 2, H, W) — two channels for dx, dy
          - SCDepthV3 fallback: (1, 1, H, W) — single depth channel

        For SCDepthV3 we compute a spatial gradient of the depth map to
        approximate horizontal motion — this is a coarse fallback.
        """
        output_key  = list(raw_outputs.keys())[0]
        output_data = raw_outputs[output_key].astype(np.float32)

        shape = output_data.shape  # e.g. (1, 2, H, W) or (1, H, W, 2)

        # Normalise to (H, W, 2) regardless of channel ordering
        if len(shape) == 4 and shape[1] == 2:
            # (1, 2, H, W) → (H, W, 2)
            flow = np.transpose(output_data[0], (1, 2, 0))
        elif len(shape) == 4 and shape[3] == 2:
            # (1, H, W, 2) → (H, W, 2)
            flow = output_data[0]
        elif len(shape) == 4 and shape[1] == 1:
            # SCDepthV3 fallback — single depth channel (1, 1, H, W)
            depth = output_data[0, 0]
            dy, dx = np.gradient(depth)
            flow = np.stack([dx, dy], axis=-1)
        elif len(shape) == 4 and shape[3] == 1:
            # SCDepthV3 (1, H, W, 1)
            depth = output_data[0, :, :, 0]
            dy, dx = np.gradient(depth)
            flow = np.stack([dx, dy], axis=-1)
        else:
            # Unknown shape — return zero with low confidence
            return self._zero_result(latency_ms=latency_ms)

        # Mean flow vector across the frame (pixels per frame)
        mean_dx = float(np.mean(flow[:, :, 0]))
        mean_dy = float(np.mean(flow[:, :, 1]))

        # Confidence: inverse of flow magnitude variance
        # High variance = inconsistent flow = low confidence (e.g. rotating scene)
        var = float(np.var(flow[:, :, 0]) + np.var(flow[:, :, 1]))
        magnitude = np.sqrt(mean_dx**2 + mean_dy**2)
        confidence = float(np.clip(1.0 / (1.0 + var * 0.1), 0.0, 1.0))

        if magnitude < MIN_FLOW_MAGNITUDE:
            # Below noise floor — drone is likely stationary
            return {
                "vx": 0.0, "vy": 0.0,
                "confidence": confidence,
                "latency_ms": latency_ms,
                "valid": True,
            }

        # Convert pixel displacement per frame to metric velocity
        # vx = (dx_pixels / fps) * (agl / focal_px)
        scale = (1.0 / self._fps) * (agl / self._focal_px)
        vx = mean_dx * scale
        vy = mean_dy * scale

        return {
            "vx":         float(vx),
            "vy":         float(vy),
            "confidence": confidence,
            "latency_ms": latency_ms,
            "valid":      True,
        }

    def _zero_result(self, latency_ms: float = 0.0) -> dict:
        return {
            "vx": 0.0, "vy": 0.0,
            "confidence": 0.0,
            "latency_ms": latency_ms,
            "valid": False,
        }

    def _update_fps(self) -> None:
        now = time.monotonic()
        self._fps_buf.append(now)
        if len(self._fps_buf) > 30:
            self._fps_buf.pop(0)
