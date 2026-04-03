#!/usr/bin/env python3
"""
foxglove_cloud_rotation.py — Foxglove cloud recording rotation manager.

Policy
------
  - Keep at most MAX_CLOUD_FLIGHTS (5) routine recordings in cloud at any time.
  - A recording is "routine" if it does NOT have the metadata key "archived"="true".
  - If the routine count exceeds MAX_CLOUD_FLIGHTS after a new upload, delete the
    oldest DELETE_BATCH (2) routine recordings — oldest by recording start time.
  - "Archived" recordings are permanently exempt and never touched by this module.
  - Deletion only runs after local postprocessing is confirmed complete, enforced
    by the caller (postprocess_mesh.py checks PLY existence before calling).
  - 2 GB soft ceiling: the module also logs a warning if estimated cloud usage
    exceeds WARN_GB — it does not auto-delete beyond the flight count policy.

Source: https://docs.foxglove.dev/api
        GET  /v1/recordings  — list all recordings for a device
        DELETE /v1/recordings/{id} — permanently delete one recording

Design
------
  Separation of concerns:
    FoxgloveCloudRotation  — pure logic: decides what to delete given a list
    FoxgloveRecordingClient — HTTP: talks to Foxglove API
    run_rotation()          — orchestrator: fetch → decide → delete → report

  This module is imported by postprocess_mesh.py. It is NOT a ROS 2 node.
  Uses stdlib urllib only — no requests/httpx dependency.

Usage (called from postprocess_mesh.py)
---------------------------------------
  from foxglove_cloud_rotation import run_rotation

  run_rotation(
      api_key     = os.environ["FOXGLOVE_API_KEY"],
      device_name = "dronepi",
  )

  The call is wrapped in try/except in postprocess_mesh.py so a rotation
  failure never blocks or fails the pipeline. Local data is always safe.
"""

import json
import os
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

# ── policy constants ───────────────────────────────────────────────────────────

MAX_CLOUD_FLIGHTS = 5      # maximum routine recordings kept in cloud
DELETE_BATCH      = 2      # how many to delete when ceiling is exceeded
WARN_GB           = 2.0    # log warning if estimated cloud usage exceeds this

FOXGLOVE_API_BASE = "https://api.foxglove.dev/v1"
_DEFAULT_TIMEOUT  = 30

# ── data structures ────────────────────────────────────────────────────────────

@dataclass
class CloudRecording:
    """
    Represents one recording returned by GET /v1/recordings.

    Fields mapped from Foxglove API response:
      id         : Foxglove-assigned recording ID (e.g. "rec_abc123")
      path       : filename / path as shown in Foxglove (e.g. "scan_20260329_091400")
      start_time : recording start as ISO 8601 string — used for sort order
      end_time   : recording end as ISO 8601 string
      size_bytes : total bytes stored in cloud (may be None if not reported)
      archived   : True if metadata key "archived" == "true" — exempt from rotation
      metadata   : raw metadata dict from the API response
    """
    id:         str
    path:       str
    start_time: str           # ISO 8601 UTC
    end_time:   str
    size_bytes: Optional[int] = None
    archived:   bool          = False
    metadata:   dict          = field(default_factory=dict)

    @property
    def start_dt(self) -> datetime:
        """Parse start_time to a timezone-aware datetime for sorting."""
        s = self.start_time
        # Handle both "Z" suffix and "+00:00" offset
        if s.endswith("Z"):
            s = s[:-1] + "+00:00"
        return datetime.fromisoformat(s)

    @property
    def size_mb(self) -> float:
        if self.size_bytes is None:
            return 0.0
        return self.size_bytes / (1024 * 1024)

    def __repr__(self) -> str:
        arch = " [ARCHIVED]" if self.archived else ""
        return (
            f"CloudRecording(id={self.id!r}, path={self.path!r}, "
            f"start={self.start_time[:10]}, size={self.size_mb:.0f}MB{arch})"
        )


@dataclass
class RotationResult:
    """
    Summary of one rotation run — returned by run_rotation().

    Fields
    ------
    routine_before  : number of routine recordings before rotation
    routine_after   : number of routine recordings after rotation
    deleted         : list of CloudRecording objects that were deleted
    archived_count  : number of archived recordings (never touched)
    skipped         : True if no rotation was needed (count <= MAX_CLOUD_FLIGHTS)
    warned_storage  : True if estimated storage exceeded WARN_GB
    errors          : list of error messages from failed delete attempts
    """
    routine_before: int
    routine_after:  int
    deleted:        list[CloudRecording] = field(default_factory=list)
    archived_count: int   = 0
    skipped:        bool  = False
    warned_storage: bool  = False
    errors:         list[str] = field(default_factory=list)


# ── HTTP client ────────────────────────────────────────────────────────────────

class FoxgloveAPIError(RuntimeError):
    def __init__(self, status: int, body: str, url: str):
        super().__init__(f"Foxglove API {status} at {url}: {body}")
        self.status = status


class FoxgloveRecordingClient:
    """
    Minimal HTTP client scoped to recording list and delete operations.

    Source: https://docs.foxglove.dev/api
      GET    /v1/recordings?deviceName=dronepi&limit=100
      DELETE /v1/recordings/{id}

    Rate limit handling: backs off on 429 using Retry-After header.
    """

    def __init__(self, api_key: str, base_url: str = FOXGLOVE_API_BASE):
        if not api_key:
            raise ValueError(
                "api_key is required. Set FOXGLOVE_API_KEY environment variable.\n"
                "Create a key with recordings.read + recordings.delete capabilities\n"
                "at: app.foxglove.dev → Settings → API Keys"
            )
        self._api_key = api_key
        self._base    = base_url.rstrip("/")

    def _headers(self) -> dict:
        return {
            "Authorization": f"Bearer {self._api_key}",
            "Content-Type":  "application/json",
        }

    def _request(self, method: str, path: str, timeout: int = _DEFAULT_TIMEOUT) -> dict:
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, headers=self._headers(), method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read().decode()
                return json.loads(raw) if raw.strip() else {}
        except urllib.error.HTTPError as exc:
            body = exc.read().decode(errors="replace")
            if exc.code == 429:
                wait = int(exc.headers.get("Retry-After", "5"))
                print(f"[cloud_rotation] Rate limited — waiting {wait}s")
                time.sleep(wait)
                with urllib.request.urlopen(req, timeout=timeout) as resp2:
                    raw = resp2.read().decode()
                    return json.loads(raw) if raw.strip() else {}
            raise FoxgloveAPIError(exc.code, body, url) from exc

    def list_recordings(self, device_name: str) -> list[CloudRecording]:
        """
        Fetch all recordings for a device, returning CloudRecording objects.

        Handles pagination: Foxglove defaults to 2000 items per page; for a
        drone with 5 cloud flights this is always one page.

        Source: GET /v1/recordings?deviceName={name}&limit=2000
        """
        path = f"/recordings?deviceName={device_name}&limit=2000"
        resp = self._request("GET", path)

        recordings = []
        for r in resp.get("recordings", []):
            # Extract metadata — the API returns it as a flat dict
            meta = r.get("metadata", {}) or {}

            # Archived flag: set manually via app.foxglove.dev or API
            # Convention: metadata key "archived" = "true"
            archived = str(meta.get("archived", "false")).lower() == "true"

            # Size: API may return sizeBytes or size_bytes depending on version
            size = r.get("sizeBytes") or r.get("size_bytes")

            recordings.append(CloudRecording(
                id         = r["id"],
                path       = r.get("path", r.get("name", r["id"])),
                start_time = r.get("startTime", r.get("start_time", "")),
                end_time   = r.get("endTime",   r.get("end_time",   "")),
                size_bytes = int(size) if size else None,
                archived   = archived,
                metadata   = meta,
            ))

        return recordings

    def delete_recording(self, recording_id: str) -> bool:
        """
        Permanently delete a recording and all its associated data.

        Source: DELETE /v1/recordings/{id}
        Returns True on success, False on failure (caller logs the error).

        WARNING: This is PERMANENT and CANNOT be undone.
        Only called by run_rotation() after all safety checks pass.
        """
        try:
            self._request("DELETE", f"/recordings/{recording_id}")
            return True
        except FoxgloveAPIError as exc:
            print(f"[cloud_rotation] Delete failed for {recording_id}: {exc}")
            return False


# ── rotation logic (pure — no HTTP calls) ─────────────────────────────────────

class FoxgloveCloudRotation:
    """
    Pure rotation logic — decides which recordings to delete given a list.

    No HTTP calls in this class. Fully testable without API access.

    Policy (re-stated precisely):
      1. Separate recordings into routine vs archived.
      2. If routine count <= MAX_CLOUD_FLIGHTS: no deletion needed.
      3. If routine count > MAX_CLOUD_FLIGHTS: sort routine by start_time
         ascending (oldest first), select the first DELETE_BATCH for deletion.
      4. Never touch archived recordings.
      5. Warn if total estimated storage (all recordings) exceeds WARN_GB.
    """

    def __init__(
        self,
        max_flights:  int   = MAX_CLOUD_FLIGHTS,
        delete_batch: int   = DELETE_BATCH,
        warn_gb:      float = WARN_GB,
    ):
        self.max_flights  = max_flights
        self.delete_batch = delete_batch
        self.warn_gb      = warn_gb

    def decide(
        self,
        recordings: list[CloudRecording],
    ) -> tuple[list[CloudRecording], bool]:
        """
        Given a list of all cloud recordings for a device, return:
          (to_delete: list[CloudRecording], warn_storage: bool)

        to_delete is empty if no rotation is needed.
        warn_storage is True if estimated total exceeds warn_gb.
        """
        routine  = [r for r in recordings if not r.archived]
        archived = [r for r in recordings if r.archived]

        # Storage warning check
        total_bytes = sum(r.size_bytes or 0 for r in recordings)
        total_gb    = total_bytes / (1024 ** 3)
        warn_storage = total_gb >= self.warn_gb

        # No rotation needed
        if len(routine) <= self.max_flights:
            return [], warn_storage

        # Sort routine recordings oldest-first by start_time
        try:
            routine_sorted = sorted(routine, key=lambda r: r.start_dt)
        except Exception:
            # Fallback: sort by path (which encodes timestamp in your naming scheme)
            routine_sorted = sorted(routine, key=lambda r: r.path)

        # Select the oldest DELETE_BATCH for deletion
        to_delete = routine_sorted[:self.delete_batch]
        return to_delete, warn_storage

    def summary(self, recordings: list[CloudRecording]) -> dict:
        """Return a human-readable summary dict for logging."""
        routine  = [r for r in recordings if not r.archived]
        archived = [r for r in recordings if r.archived]
        total_mb = sum(r.size_bytes or 0 for r in recordings) / (1024 * 1024)
        return {
            "total":       len(recordings),
            "routine":     len(routine),
            "archived":    len(archived),
            "total_mb":    round(total_mb, 1),
            "at_ceiling":  len(routine) > self.max_flights,
        }


# ── orchestrator ───────────────────────────────────────────────────────────────

def run_rotation(
    api_key:     str,
    device_name: str = "dronepi",
    dry_run:     bool = False,
) -> RotationResult:
    """
    Full rotation cycle: list → decide → delete → report.

    Parameters
    ----------
    api_key     : Foxglove API key (recordings.read + recordings.delete)
    device_name : name of the device in your Foxglove org
    dry_run     : if True, log what would be deleted but make no API calls

    Returns
    -------
    RotationResult — summary of what happened

    Called by postprocess_mesh.py after PLY output is confirmed:
      from foxglove_cloud_rotation import run_rotation
      result = run_rotation(api_key=os.environ["FOXGLOVE_API_KEY"])
      print(f"  Cloud rotation: {result.routine_before} → {result.routine_after} flights")
    """
    client   = FoxgloveRecordingClient(api_key=api_key)
    rotation = FoxgloveCloudRotation()

    # ── 1. list current cloud state ───────────────────────────────────────────
    recordings = client.list_recordings(device_name=device_name)
    summary    = rotation.summary(recordings)

    print(f"[cloud_rotation] Cloud state for '{device_name}':")
    print(f"  routine={summary['routine']}  archived={summary['archived']}"
          f"  total_mb={summary['total_mb']:.0f}  ceiling={MAX_CLOUD_FLIGHTS}")

    # ── 2. decide what to delete ──────────────────────────────────────────────
    to_delete, warn_storage = rotation.decide(recordings)

    routine_before = summary["routine"]

    if warn_storage:
        print(f"[cloud_rotation] WARNING: estimated cloud usage "
              f"{summary['total_mb'] / 1024:.2f} GB exceeds {WARN_GB} GB soft ceiling")

    if not to_delete:
        print(f"[cloud_rotation] No rotation needed "
              f"({routine_before} <= {MAX_CLOUD_FLIGHTS} flights)")
        return RotationResult(
            routine_before = routine_before,
            routine_after  = routine_before,
            archived_count = summary["archived"],
            skipped        = True,
            warned_storage = warn_storage,
        )

    print(f"[cloud_rotation] Ceiling exceeded — will delete {len(to_delete)} oldest routine recording(s):")
    for r in to_delete:
        print(f"  DELETE: {r.path}  start={r.start_time[:19]}  size={r.size_mb:.0f}MB")

    # ── 3. delete ─────────────────────────────────────────────────────────────
    deleted = []
    errors  = []

    for rec in to_delete:
        if dry_run:
            print(f"  [DRY RUN] would delete: {rec.id}  ({rec.path})")
            deleted.append(rec)
            continue

        ok = client.delete_recording(rec.id)
        if ok:
            deleted.append(rec)
            print(f"  [cloud_rotation] Deleted: {rec.path} ({rec.id})")
        else:
            errors.append(f"Failed to delete {rec.id} ({rec.path})")

        # Courtesy delay between deletes to avoid hammering the API
        time.sleep(0.5)

    routine_after = routine_before - len(deleted)

    print(f"[cloud_rotation] Rotation complete: "
          f"{routine_before} → {routine_after} routine recordings in cloud")

    return RotationResult(
        routine_before = routine_before,
        routine_after  = routine_after,
        deleted        = deleted,
        archived_count = summary["archived"],
        skipped        = False,
        warned_storage = warn_storage,
        errors         = errors,
    )


# ── convenience factory ───────────────────────────────────────────────────────

def get_rotation_from_env(dry_run: bool = False) -> RotationResult:
    """
    Run rotation using FOXGLOVE_API_KEY and FOXGLOVE_DEVICE_NAME env vars.
    Raises ValueError if FOXGLOVE_API_KEY is not set.
    """
    api_key = os.environ.get("FOXGLOVE_API_KEY", "")
    if not api_key:
        raise ValueError("FOXGLOVE_API_KEY is not set — rotation skipped")
    device  = os.environ.get("FOXGLOVE_DEVICE_NAME", "dronepi")
    return run_rotation(api_key=api_key, device_name=device, dry_run=dry_run)
