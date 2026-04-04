#!/usr/bin/env python3
"""
test_system_sound_language_mavros_v3.py
--------------------------------------
Standalone sound-language tester using MAVROS PlayTuneV2.

This v3 uses the selected tunes from the approved tune library.

Usage
-----
source /opt/ros/jazzy/setup.bash
conda activate dronepi

# Interactive menu
python3 test_system_sound_language_mavros_v3.py

# Full demo once
python3 test_system_sound_language_mavros_v3.py --demo

# Loop demo
python3 test_system_sound_language_mavros_v3.py --loop

# Single custom tune
python3 test_system_sound_language_mavros_v3.py --tune "T200L12O5CEG>C" --label "scan_start"
"""

import argparse
import sys
import time


TUNES: dict[str, str] = {
    "SYSTEM_START":       "T180L8O4CEGCEG>C",   # start_v3_fanfare
    "SCAN_READY":         "T250L32O5CE",        # ack_v4_chirp
    "SCAN_START":         "T200L12O5CEG>C",     # start_v2_slower
    "SCAN_ACTIVE":        "T200L64O4GE",        # proc_v4_twotick
    "SCAN_FINISHED":      "T180L8O5C<GEG<C",    # stop_v3_twostep
    "POSTPROCESS_ACTIVE": "T220L64O5C",         # proc_v5_hightick
    "POSTPROCESS_DONE":   "T180L12O5CEGCE",     # done_v2_chord
    "WARNING":            "T200L32O5CC",        # ack_v5_double
    "ERROR":              "T140L4O4C<C",        # error_v3_urgent
    "CRITICAL":           "T180L8O6C<A<F",      # error_v2_descend
    "SYSTEM_FAILURE":     "T220L8O6CECECE",     # error_v5_alarm
}

DEMO_SEQUENCE = [
    ("SYSTEM_START", 1, 2.0),
    ("SCAN_READY", 1, 1.0),
    ("SCAN_START", 1, 1.0),
    ("SCAN_ACTIVE", 5, 1.0),
    ("SCAN_FINISHED", 1, 1.2),
    ("POSTPROCESS_ACTIVE", 5, 1.0),
    ("POSTPROCESS_DONE", 1, 2.0),
    ("WARNING", 1, 1.0),
    ("ERROR", 1, 1.0),
    ("CRITICAL", 1, 2.0),
    ("SYSTEM_FAILURE", 1, 2.0),
]


def make_publisher():
    try:
        import rclpy
        from mavros_msgs.msg import PlayTuneV2
    except ImportError as exc:
        print(f"[ERROR] ROS 2 / mavros_msgs not available: {exc}")
        print("Source ROS 2 and activate the dronepi environment first.")
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("system_sound_language_tester_v3")
    pub = node.create_publisher(PlayTuneV2, "/mavros/play_tune", 10)
    time.sleep(0.5)
    return node, pub, PlayTuneV2, rclpy


def play_tune(pub, PlayTuneV2, tune: str, label: str) -> None:
    print(f"\n🔊 Playing: {label}")
    msg = PlayTuneV2()
    msg.format = 1
    msg.tune = tune
    pub.publish(msg)


def run_demo(pub, PlayTuneV2) -> None:
    print("\n🚀 Starting FULL SYSTEM SOUND DEMO (v3)\n")

    for name, repeat_count, gap_s in DEMO_SEQUENCE:
        tune = TUNES[name]

        if name == "SCAN_ACTIVE":
            print("\n🔁 Scanning active...")
        elif name == "POSTPROCESS_ACTIVE":
            print("\n🧠 Post-processing...")

        for idx in range(repeat_count):
            suffix = f" {idx + 1}" if repeat_count > 1 else ""
            play_tune(pub, PlayTuneV2, tune, f"{name}{suffix}")
            time.sleep(gap_s)

    print("\n✅ Demo complete\n")


def run_single(pub, PlayTuneV2, tune: str, label: str) -> None:
    play_tune(pub, PlayTuneV2, tune, label)
    time.sleep(0.5)


def run_menu(pub, PlayTuneV2) -> None:
    keys = list(TUNES.keys())

    print("\n" + "=" * 72)
    print("  DronePi System Sound Language Tester v3")
    print("  MAVROS PlayTuneV2 / QBASIC Format 1")
    print("=" * 72)
    print("  Commands:")
    print("    <number>   Play that tune")
    print("    d          Play full demo sequence")
    print("    a          Play all single tunes once")
    print("    q          Quit")
    print("=" * 72 + "\n")

    for i, key in enumerate(keys):
        print(f"    [{i:2d}]  {key:<20} {TUNES[key]}")

    while True:
        try:
            raw = input("\nSelect tune > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            return

        if raw == "q":
            print("Exiting.")
            return

        if raw == "d":
            run_demo(pub, PlayTuneV2)
            continue

        if raw == "a":
            for key in keys:
                play_tune(pub, PlayTuneV2, TUNES[key], key)
                time.sleep(0.8)
            print("\nDone.\n")
            continue

        try:
            idx = int(raw)
        except ValueError:
            print("Invalid input. Enter a number, d, a, or q.")
            continue

        if idx < 0 or idx >= len(keys):
            print(f"Index out of range (0-{len(keys) - 1}).")
            continue

        key = keys[idx]
        play_tune(pub, PlayTuneV2, TUNES[key], key)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test the semantic DronePi sound language using MAVROS."
    )
    parser.add_argument("--loop", action="store_true", help="Loop the full demo sequence.")
    parser.add_argument("--demo", action="store_true", help="Run the full demo sequence once.")
    parser.add_argument("--tune", type=str, help='Play a single QBASIC tune string and exit.')
    parser.add_argument("--label", type=str, default="custom", help="Label for --tune.")
    args = parser.parse_args()

    node, pub, PlayTuneV2, rclpy = make_publisher()

    try:
        if args.tune:
            run_single(pub, PlayTuneV2, args.tune, args.label)
        elif args.loop:
            while True:
                run_demo(pub, PlayTuneV2)
                print("🔁 Restarting in 3 seconds...\n")
                time.sleep(3.0)
        elif args.demo:
            run_demo(pub, PlayTuneV2)
        else:
            run_menu(pub, PlayTuneV2)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
