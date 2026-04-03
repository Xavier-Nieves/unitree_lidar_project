#!/usr/bin/env python3
"""
test_foxglove_cloud_rotation.py — Standalone test suite for foxglove_cloud_rotation.py

This is a TEST SCRIPT, not a production module.
  - NOT imported by postprocess_mesh.py or any other module.
  - Runs standalone from the command line.
  - Dry-run mode by default (no real API calls unless --live is passed).

Usage
-----
  # Dry-run (no API calls, no API key needed):
  python3 test_foxglove_cloud_rotation.py

  # Live dry-run (lists your real cloud recordings, deletes nothing):
  FOXGLOVE_API_KEY=fox_sk_... python3 test_foxglove_cloud_rotation.py --live-dry-run

  # Full live test (will delete real recordings — use with caution):
  FOXGLOVE_API_KEY=fox_sk_... python3 test_foxglove_cloud_rotation.py --live
"""

import sys
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parent))

from foxglove_cloud_rotation import (
    CloudRecording,
    FoxgloveCloudRotation,
    RotationResult,
    run_rotation,
    MAX_CLOUD_FLIGHTS,
    DELETE_BATCH,
    WARN_GB,
)

# ── test helpers ──────────────────────────────────────────────────────────────

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
INFO = "\033[94m[INFO]\033[0m"

_tests = _fails = 0

def assert_eq(label, actual, expected):
    global _tests, _fails
    _tests += 1
    if actual == expected:
        print(f"  {PASS} {label}")
    else:
        _fails += 1
        print(f"  {FAIL} {label}")
        print(f"         expected: {expected!r}")
        print(f"         got:      {actual!r}")

def assert_true(label, cond):
    global _tests, _fails
    _tests += 1
    if cond:
        print(f"  {PASS} {label}")
    else:
        _fails += 1
        print(f"  {FAIL} {label}")

def section(title):
    print(f"\n{'─'*60}\n  {title}\n{'─'*60}")


# ── fixture factory ───────────────────────────────────────────────────────────

def _make_recording(
    n:         int,
    archived:  bool = False,
    size_mb:   float = 120.0,
    days_ago:  int = None,
) -> CloudRecording:
    """
    Create a synthetic CloudRecording for testing.
    n is used to generate a unique id and deterministic timestamps.
    """
    if days_ago is None:
        days_ago = 10 - n   # recording 1 is oldest (9 days ago), recording 10 is newest (0 days ago)
    start_dt  = datetime.now(timezone.utc) - timedelta(days=days_ago)
    end_dt    = start_dt + timedelta(minutes=4)
    meta      = {"archived": "true"} if archived else {}
    return CloudRecording(
        id         = f"rec_{n:03d}",
        path       = f"scan_2026032{n:01d}_09140{n:01d}",
        start_time = start_dt.isoformat().replace("+00:00", "Z"),
        end_time   = end_dt.isoformat().replace("+00:00", "Z"),
        size_bytes = int(size_mb * 1024 * 1024),
        archived   = archived,
        metadata   = meta,
    )


def _make_fleet(
    routine_count: int,
    archived_count: int = 0,
    size_mb: float = 120.0,
) -> list[CloudRecording]:
    """Build a synthetic fleet of recordings for testing."""
    recordings = []
    for i in range(1, routine_count + 1):
        recordings.append(_make_recording(i, archived=False, size_mb=size_mb))
    for i in range(routine_count + 1, routine_count + archived_count + 1):
        recordings.append(_make_recording(i, archived=True, size_mb=size_mb))
    return recordings


# ── test suites ───────────────────────────────────────────────────────────────

def test_constants():
    section("Policy constants")
    assert_eq("MAX_CLOUD_FLIGHTS == 5",  MAX_CLOUD_FLIGHTS, 5)
    assert_eq("DELETE_BATCH == 2",        DELETE_BATCH,       2)
    assert_true("WARN_GB is numeric",     isinstance(WARN_GB, float))
    assert_true("WARN_GB around 2.0",     1.5 <= WARN_GB <= 3.0)


def test_no_rotation_needed():
    section("No rotation needed — fleet at or below ceiling")
    rotation = FoxgloveCloudRotation()

    # Exactly at ceiling
    recordings = _make_fleet(5)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("5 routine → no deletion", to_delete, [])

    # Below ceiling
    recordings = _make_fleet(3)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("3 routine → no deletion", to_delete, [])

    # Empty fleet
    to_delete, _ = rotation.decide([])
    assert_eq("0 recordings → no deletion", to_delete, [])


def test_rotation_triggered():
    section("Rotation triggered — fleet exceeds ceiling")
    rotation = FoxgloveCloudRotation()

    # One over ceiling: 6 routine → delete 2 oldest
    recordings = _make_fleet(6)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("6 routine → delete 2",   len(to_delete), DELETE_BATCH)

    # Deleted are the 2 oldest (smallest days_ago value → earliest start_time)
    # _make_recording(1) is oldest (days_ago=9), rec(2) is second oldest (days_ago=8)
    deleted_ids = {r.id for r in to_delete}
    assert_true("Oldest rec_001 is selected", "rec_001" in deleted_ids)
    assert_true("Second oldest rec_002 is selected", "rec_002" in deleted_ids)

    # Well over ceiling: 9 routine → still only delete DELETE_BATCH (2)
    recordings = _make_fleet(9)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("9 routine → still delete exactly 2", len(to_delete), DELETE_BATCH)


def test_archived_never_deleted():
    section("Archived recordings are never deleted")
    rotation = FoxgloveCloudRotation()

    # 5 routine + 3 archived = 8 total, but only 5 routine
    # → ceiling not exceeded for routine → no deletion
    recordings = _make_fleet(routine_count=5, archived_count=3)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("5 routine + 3 archived → no routine deletion", to_delete, [])

    # 7 routine + 2 archived → routine exceeds ceiling → delete 2 ROUTINE oldest
    recordings = _make_fleet(routine_count=7, archived_count=2)
    to_delete, _ = rotation.decide(recordings)
    assert_eq("7 routine + 2 archived → delete 2 routine", len(to_delete), DELETE_BATCH)

    # Verify none of the deleted are archived
    any_archived = any(r.archived for r in to_delete)
    assert_true("None of the deleted recordings are archived", not any_archived)

    # ALL routine, archived ones mixed in: none of the archived IDs appear in to_delete
    routine_ids  = {r.id for r in recordings if not r.archived}
    archived_ids = {r.id for r in recordings if r.archived}
    deleted_ids  = {r.id for r in to_delete}
    assert_true("Deleted IDs are subset of routine IDs", deleted_ids.issubset(routine_ids))
    assert_true("No archived IDs in deleted set", deleted_ids.isdisjoint(archived_ids))


def test_storage_warning():
    section("Storage warning at WARN_GB threshold")
    rotation = FoxgloveCloudRotation(warn_gb=2.0)

    # 5 flights × 120 MB = 600 MB → no warning
    recordings = _make_fleet(5, size_mb=120.0)
    _, warn = rotation.decide(recordings)
    assert_true("600 MB total → no warning", not warn)

    # 5 flights × 450 MB = 2250 MB = ~2.2 GB → warning
    recordings = _make_fleet(5, size_mb=450.0)
    _, warn = rotation.decide(recordings)
    assert_true("2.2 GB total → warning triggered", warn)

    # Warning fires even when rotation is not needed (count at ceiling)
    recordings = _make_fleet(5, size_mb=500.0)
    to_delete, warn = rotation.decide(recordings)
    assert_eq("At ceiling with large files: no deletion", to_delete, [])
    assert_true("But storage warning still fires", warn)


def test_summary():
    section("Summary statistics")
    rotation = FoxgloveCloudRotation()

    recordings = _make_fleet(routine_count=4, archived_count=2, size_mb=200.0)
    s = rotation.summary(recordings)
    assert_eq("total == 6",      s["total"],    6)
    assert_eq("routine == 4",    s["routine"],  4)
    assert_eq("archived == 2",   s["archived"], 2)
    assert_true("total_mb ≈ 1200", 1180 <= s["total_mb"] <= 1220)
    assert_eq("at_ceiling False (4 <= 5)", s["at_ceiling"], False)

    recordings = _make_fleet(routine_count=6)
    s = rotation.summary(recordings)
    assert_eq("at_ceiling True (6 > 5)", s["at_ceiling"], True)


def test_cloud_recording_properties():
    section("CloudRecording dataclass")
    rec = _make_recording(1)
    assert_true("start_dt returns datetime",        isinstance(rec.start_dt, datetime))
    assert_true("start_dt is timezone-aware",       rec.start_dt.tzinfo is not None)
    assert_true("size_mb > 0",                      rec.size_mb > 0)
    assert_eq  ("archived default is False",         rec.archived, False)
    assert_true("__repr__ contains id",              "rec_001" in repr(rec))

    # Archived recording
    rec_arch = _make_recording(2, archived=True)
    assert_eq("archived rec has archived=True", rec_arch.archived, True)
    assert_true("[ARCHIVED] in repr",            "[ARCHIVED]" in repr(rec_arch))


def test_run_rotation_mocked():
    section("run_rotation() with mocked API client")
    from unittest.mock import patch as mock_patch

    recordings = _make_fleet(routine_count=6, archived_count=1)
    delete_calls = []

    def fake_list(device_name):
        return recordings

    def fake_delete(recording_id):
        delete_calls.append(recording_id)
        return True

    with mock_patch("foxglove_cloud_rotation.FoxgloveRecordingClient") as MockClient:
        instance = MockClient.return_value
        instance.list_recordings.side_effect  = fake_list
        instance.delete_recording.side_effect = fake_delete

        result = run_rotation(api_key="fox_sk_test", device_name="dronepi")

    assert_eq("routine_before == 6",              result.routine_before, 6)
    assert_eq("routine_after == 4",               result.routine_after,  4)
    assert_eq("2 recordings deleted",             len(result.deleted),   DELETE_BATCH)
    assert_eq("2 delete API calls made",          len(delete_calls),     DELETE_BATCH)
    assert_eq("no errors",                        result.errors,         [])
    assert_eq("skipped == False",                 result.skipped,        False)
    assert_eq("archived_count == 1",              result.archived_count, 1)

    # Verify oldest two were deleted
    deleted_ids = {r.id for r in result.deleted}
    assert_true("rec_001 deleted (oldest)", "rec_001" in deleted_ids)
    assert_true("rec_002 deleted (2nd oldest)", "rec_002" in deleted_ids)


def test_run_rotation_dry_run():
    section("run_rotation() dry_run mode — no deletes executed")
    from unittest.mock import patch as mock_patch

    recordings = _make_fleet(routine_count=7)
    delete_calls = []

    with mock_patch("foxglove_cloud_rotation.FoxgloveRecordingClient") as MockClient:
        instance = MockClient.return_value
        instance.list_recordings.return_value = recordings
        instance.delete_recording.side_effect = lambda x: delete_calls.append(x) or True

        result = run_rotation(api_key="fox_sk_test", dry_run=True)

    assert_eq("dry_run: 0 real API delete calls", len(delete_calls), 0)
    assert_eq("dry_run: still reports 2 'would delete'", len(result.deleted), DELETE_BATCH)
    assert_eq("dry_run: routine_after reflects simulated deletion",
              result.routine_after, 7 - DELETE_BATCH)


def test_missing_api_key():
    section("Error handling — missing API key")
    try:
        run_rotation(api_key="")
        _fails += 1
        print(f"  {FAIL} Empty key should raise ValueError")
    except ValueError as e:
        assert_true("ValueError raised for empty key",      True)
        assert_true("Message mentions FOXGLOVE_API_KEY", "recordings" in str(e) or "api_key" in str(e).lower())


# ── live tests ────────────────────────────────────────────────────────────────

def test_live_dry_run():
    """List your real cloud recordings, decide what would be deleted, but delete nothing."""
    import os
    section("LIVE DRY RUN — lists real recordings, deletes nothing")

    api_key = os.environ.get("FOXGLOVE_API_KEY", "")
    if not api_key:
        print(f"  {INFO} FOXGLOVE_API_KEY not set — skipping live dry-run")
        return

    device = os.environ.get("FOXGLOVE_DEVICE_NAME", "dronepi")
    print(f"  {INFO} Connecting to Foxglove for device '{device}' ...")

    try:
        result = run_rotation(api_key=api_key, device_name=device, dry_run=True)
        assert_true("live dry-run returned RotationResult", isinstance(result, RotationResult))
        print(f"  {INFO} routine_before={result.routine_before}  "
              f"archived={result.archived_count}  "
              f"would_delete={len(result.deleted)}")
    except Exception as e:
        global _fails
        _fails += 1
        print(f"  {FAIL} Live API call failed: {e}")


def test_live_full(device_name: str = "dronepi"):
    """Actually run rotation — will permanently delete recordings if count > 5."""
    import os
    section("LIVE FULL — real deletions will occur if count > MAX_CLOUD_FLIGHTS")

    api_key = os.environ.get("FOXGLOVE_API_KEY", "")
    if not api_key:
        print(f"  {INFO} FOXGLOVE_API_KEY not set — skipping")
        return

    print(f"  {INFO} WARNING: this will permanently delete recordings if count > {MAX_CLOUD_FLIGHTS}")
    confirm = input("  Type 'yes' to continue: ")
    if confirm.strip().lower() != "yes":
        print(f"  {INFO} Aborted by user")
        return

    try:
        result = run_rotation(api_key=api_key, device_name=device_name, dry_run=False)
        assert_true("Result is RotationResult", isinstance(result, RotationResult))
        print(f"  {INFO} Deleted: {len(result.deleted)}  Errors: {len(result.errors)}")
        assert_eq("no errors during live run", result.errors, [])
    except Exception as e:
        global _fails
        _fails += 1
        print(f"  {FAIL} Live run failed: {e}")


# ── entrypoint ────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Standalone test suite for foxglove_cloud_rotation.py"
    )
    parser.add_argument("--live-dry-run", action="store_true",
                        help="List real cloud recordings, decide but delete nothing")
    parser.add_argument("--live", action="store_true",
                        help="Run full rotation against real cloud (may delete recordings)")
    args = parser.parse_args()

    print("\n" + "="*60)
    print("  foxglove_cloud_rotation.py — Test Suite")
    print("="*60)

    test_constants()
    test_cloud_recording_properties()
    test_no_rotation_needed()
    test_rotation_triggered()
    test_archived_never_deleted()
    test_storage_warning()
    test_summary()
    test_run_rotation_mocked()
    test_run_rotation_dry_run()
    test_missing_api_key()

    if args.live_dry_run or args.live:
        test_live_dry_run()
    if args.live:
        test_live_full()

    print(f"\n{'='*60}")
    passed = _tests - _fails
    print(f"  Results: {passed}/{_tests} passed")
    if _fails == 0:
        print(f"  {PASS} All tests passed")
    else:
        print(f"  {FAIL} {_fails} test(s) FAILED")
    print("="*60 + "\n")
    sys.exit(0 if _fails == 0 else 1)


if __name__ == "__main__":
    main()
