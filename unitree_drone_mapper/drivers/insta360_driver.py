"""Insta360 X3 WiFi HTTP API client.

Controls the Insta360 X3 camera over its WiFi HTTP API for:
- Taking photos (equirectangular)
- Starting/stopping video
- Downloading media
- Camera status queries

The camera creates a WiFi hotspot that the Pi connects to.
"""

import time
import os
import requests
from pathlib import Path
from typing import Optional

from utils.logger import setup_logger

logger = setup_logger(__name__)

# Insta360 X3 WiFi API defaults
DEFAULT_BASE_URL = "http://192.168.42.1"
API_TIMEOUT = 10  # seconds


class Insta360Driver:
    """HTTP API client for Insta360 X3 camera."""

    def __init__(self, config: dict):
        """Initialize Insta360 driver.

        Args:
            config: Camera config dict from config.yaml (camera section).
        """
        self.base_url = DEFAULT_BASE_URL
        self.wifi_ssid = config.get("wifi_ssid", "Insta360_X3_XXXX")
        self.resolution = config.get("resolution", [5760, 2880])
        self.jpeg_quality = config.get("jpeg_quality", 95)
        self._connected = False

    def connect(self) -> bool:
        """Test connection to the camera.

        Returns:
            True if camera is reachable.
        """
        try:
            resp = requests.get(
                f"{self.base_url}/osc/info",
                timeout=API_TIMEOUT,
            )
            if resp.status_code == 200:
                info = resp.json()
                logger.info(
                    f"Connected to Insta360: {info.get('model', 'unknown')} "
                    f"(FW: {info.get('firmwareVersion', '?')})"
                )
                self._connected = True
                return True
        except requests.ConnectionError:
            logger.warning(
                f"Cannot reach Insta360 at {self.base_url}. "
                f"Ensure WiFi is connected to {self.wifi_ssid}"
            )
        except requests.Timeout:
            logger.warning("Insta360 connection timed out")
        self._connected = False
        return False

    def take_photo(self) -> Optional[str]:
        """Capture an equirectangular photo.

        Returns:
            URL/path of the captured image, or None on failure.
        """
        if not self._connected:
            if not self.connect():
                return None

        try:
            # Start capture
            resp = requests.post(
                f"{self.base_url}/osc/commands/execute",
                json={
                    "name": "camera.takePicture",
                    "parameters": {},
                },
                timeout=API_TIMEOUT,
            )

            if resp.status_code == 200:
                result = resp.json()
                image_url = result.get("results", {}).get("fileUrl")
                logger.info(f"Photo captured: {image_url}")
                return image_url
            else:
                logger.error(f"Capture failed: HTTP {resp.status_code}")
                return None

        except requests.RequestException as e:
            logger.error(f"Capture error: {e}")
            return None

    def download_image(self, image_url: str, output_dir: str) -> Optional[str]:
        """Download an image from the camera to local storage.

        Args:
            image_url: URL of the image on the camera.
            output_dir: Local directory to save the image.

        Returns:
            Local file path of the downloaded image, or None on failure.
        """
        try:
            resp = requests.get(image_url, timeout=30, stream=True)
            if resp.status_code == 200:
                filename = os.path.basename(image_url)
                filepath = os.path.join(output_dir, filename)
                Path(output_dir).mkdir(parents=True, exist_ok=True)

                with open(filepath, "wb") as f:
                    for chunk in resp.iter_content(chunk_size=8192):
                        f.write(chunk)

                logger.info(f"Downloaded: {filepath}")
                return filepath
        except requests.RequestException as e:
            logger.error(f"Download error: {e}")
        return None

    def capture_and_download(self, output_dir: str) -> Optional[str]:
        """Take a photo and immediately download it.

        Args:
            output_dir: Local directory to save the image.

        Returns:
            Local file path, or None on failure.
        """
        url = self.take_photo()
        if url:
            # Wait for camera to finish processing
            time.sleep(1.0)
            return self.download_image(url, output_dir)
        return None

    def get_status(self) -> dict:
        """Get camera status (battery, storage, mode, etc.).

        Returns:
            Status dict, or empty dict on failure.
        """
        try:
            resp = requests.post(
                f"{self.base_url}/osc/commands/execute",
                json={"name": "camera.getOptions", "parameters": {"optionNames": [
                    "remainingSpace", "batteryLevel", "captureMode",
                ]}},
                timeout=API_TIMEOUT,
            )
            if resp.status_code == 200:
                return resp.json().get("results", {}).get("options", {})
        except requests.RequestException:
            pass
        return {}

    def is_connected(self) -> bool:
        """Check if currently connected to the camera."""
        return self._connected
