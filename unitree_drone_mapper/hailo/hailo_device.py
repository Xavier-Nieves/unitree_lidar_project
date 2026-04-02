"""hailo/hailo_device.py — Hailo-8 PCIe device manager.

Handles device initialisation, model loading, health monitoring, and clean
shutdown. All other Hailo modules receive a HailoDevice instance rather than
managing the VDevice directly — single ownership prevents PCIe port conflicts.

Environment: hailo_inference_env
    source ~/hailo_inference_env/bin/activate

Usage:
    device = HailoDevice()
    if device.is_available():
        model = device.load_model("/usr/local/hailo/resources/models/hailo8/yolov8m.hef")
        # pass model to HailoOpticalFlow or HailoDepthEstimator
    device.shutdown()

Design notes:
    - VDevice is opened once and shared across all loaded models.
    - Model loading returns a ConfiguredNetwork handle, not a raw HEF object,
      so the caller does not need to manage configure/release lifecycle.
    - is_available() checks /dev/hailo0 at the OS level before attempting
      any hailo_platform import — safe to call without the venv active.
    - Temperature polling uses hailortcli subprocess to avoid blocking the
      inference thread. The Hailo-8 thermal limit is 95°C; log warnings
      above 80°C to match the active-cooler guidance from prior sessions.
"""

import os
import subprocess
import time
from pathlib import Path


HAILO_DEVICE_PATH = Path("/dev/hailo0")
TEMP_WARN_C       = 80.0   # Log warning above this temperature
TEMP_CRIT_C       = 90.0   # Log critical above this — consider throttling


class HailoDevice:
    """Owns the Hailo-8 VDevice and provides model loading.

    Args:
        device_path: Path to the PCIe device node. Default /dev/hailo0.

    Raises:
        RuntimeError: If hailo_platform cannot be imported or VDevice fails
                      to open. Caller should catch and fall back gracefully.
    """

    def __init__(self, device_path: str = str(HAILO_DEVICE_PATH)):
        self._device_path = Path(device_path)
        self._vdevice     = None
        self._open        = False

    # ── Public API ────────────────────────────────────────────────────────────

    def is_available(self) -> bool:
        """Return True if /dev/hailo0 exists and is accessible.

        Does not attempt to open the device — safe to call without the
        hailo_inference_env active. Used by main.py and drone_watchdog.py
        as a lightweight pre-check before launching the Hailo flight node.
        """
        return self._device_path.exists() and os.access(self._device_path, os.R_OK)

    def open(self) -> None:
        """Open the VDevice. Must be called before load_model().

        Raises:
            RuntimeError: On hailo_platform import failure or VDevice open failure.
        """
        try:
            from hailo_platform import VDevice
        except ImportError as exc:
            raise RuntimeError(
                f"hailo_platform not importable — is hailo_inference_env active? "
                f"Original error: {exc}"
            ) from exc

        try:
            self._vdevice = VDevice()
            self._open    = True
        except Exception as exc:
            raise RuntimeError(f"VDevice open failed: {exc}") from exc

    def load_model(self, hef_path: str):
        """Load and configure a compiled .hef model on the Hailo-8.

        Args:
            hef_path: Absolute path to a compiled .hef file.

        Returns:
            Configured network group handle (hailo_platform ConfiguredNetwork).

        Raises:
            RuntimeError: If the device is not open or model loading fails.
            FileNotFoundError: If hef_path does not exist.
        """
        if not self._open:
            raise RuntimeError("HailoDevice.open() must be called before load_model()")

        hef_path = Path(hef_path)
        if not hef_path.exists():
            raise FileNotFoundError(f"HEF model not found: {hef_path}")

        try:
            from hailo_platform import HEF, ConfigureParams, HailoStreamInterface
        except ImportError as exc:
            raise RuntimeError(f"hailo_platform import failed: {exc}") from exc

        hef            = HEF(str(hef_path))
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        network_groups = self._vdevice.configure(hef, configure_params)

        if not network_groups:
            raise RuntimeError(f"No network groups configured from {hef_path.name}")

        return network_groups[0]

    def get_temperature(self) -> float:
        """Return Hailo-8 die temperature in °C via hailortcli.

        Uses subprocess rather than hailo_platform API to avoid blocking
        the inference thread. Returns -1.0 on failure.
        """
        try:
            result = subprocess.run(
                ["hailortcli", "fw-control", "temperature"],
                capture_output=True, text=True, timeout=3,
            )
            # Output format: "Temperature: XX.X °C"
            for line in result.stdout.splitlines():
                if "Temperature" in line or "temperature" in line:
                    parts = line.split()
                    for p in parts:
                        try:
                            temp = float(p)
                            if 0 < temp < 120:
                                return temp
                        except ValueError:
                            continue
        except Exception:
            pass
        return -1.0

    def log_thermal_status(self) -> None:
        """Log temperature with appropriate level. Call periodically."""
        temp = self.get_temperature()
        if temp < 0:
            return  # hailortcli unavailable — skip silently
        if temp >= TEMP_CRIT_C:
            print(f"[HAILO CRITICAL] Temperature {temp:.1f}°C — approaching limit")
        elif temp >= TEMP_WARN_C:
            print(f"[HAILO WARN] Temperature {temp:.1f}°C — monitor closely")

    def shutdown(self) -> None:
        """Release the VDevice. Call once at process exit."""
        if self._vdevice is not None:
            try:
                self._vdevice.release()
            except Exception:
                pass
            self._vdevice = None
        self._open = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.shutdown()
