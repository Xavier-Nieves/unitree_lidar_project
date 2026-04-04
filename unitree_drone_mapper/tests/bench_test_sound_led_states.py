#!/usr/bin/env python3
"""
bench_test_sound_led_states.py
------------------------------
Bench-test the DronePi sound + LED language without flying.

What it does
------------
- Publishes tunes to Pixhawk through MAVROS PlayTuneV2
- Writes /tmp/watchdog_status.json so led_service_matched.py can mirror states
- Lets you FALSE-TRIGGER each event/alarm on the bench
- Lets you test armed/unarmed alarm behavior

Prereqs
-------
1) Run your matched LED service in another terminal:
   python3 led_service_matched.py

2) Source ROS 2 + environment, then run this script:
   source /opt/ros/jazzy/setup.bash
   conda activate dronepi
   python3 bench_test_sound_led_states.py
"""

from __future__ import annotations

import json
import sys
import time
import threading
from pathlib import Path
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from mavros_msgs.msg import PlayTuneV2
except ImportError as exc:
    print(f"[ERROR] Missing ROS 2 / MAVROS dependencies: {exc}")
    print("Source ROS 2 and activate your environment first.")
    sys.exit(1)


WATCHDOG_STATUS_FILE = "/tmp/watchdog_status.json"
WATCHDOG_STATUS_FILE_TMP = WATCHDOG_STATUS_FILE + ".tmp"

TUNE_FORMAT = 1

TUNE_SYSTEM_START       = "T180L8O4CEGCEG>C"
TUNE_SCAN_READY         = "T250L32O5CE"
TUNE_SCAN_START         = "T200L12O5CEG>C"
TUNE_SCAN_ACTIVE        = "T200L64O4GE"
TUNE_SCAN_FINISHED      = "T180L8O5C<GEG<C"
TUNE_POSTPROCESS_ACTIVE = "T220L64O5C"
TUNE_POSTPROCESS_DONE   = "T180L12O5CEGCE"
TUNE_WARNING            = "T200L32O5CC"

TUNE_ERROR              = "T140L4O4C<C"
TUNE_CRITICAL           = "T180L8O6C<A<F"
TUNE_SYSTEM_FAILURE     = "T220L8O6CECECE"

SCAN_BEEP_INTERVAL_S         = 1.0
POSTPROCESS_BEEP_INTERVAL_S  = 1.0
ERROR_REPEAT_INTERVAL_S      = 2.0
CRITICAL_REPEAT_INTERVAL_S   = 1.25
SYSTEM_FAILURE_INTERVAL_S    = 0.85


def _sleep_responsive(active_fn, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        if not active_fn():
            return
        time.sleep(0.1)


class PeriodicBeeper:
    def __init__(self, play_tune_fn, tune: str, interval_s: float) -> None:
        self._play = play_tune_fn
        self._tune = tune
        self._interval_s = interval_s
        self._active = False
        self._thread: Optional[threading.Thread] = None

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._active = False
        if self._thread:
            self._thread.join(timeout=self._interval_s + 1.0)
            self._thread = None

    def _loop(self) -> None:
        while self._active:
            try:
                self._play(self._tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._active, self._interval_s)


class ScanBeeper(PeriodicBeeper):
    def __init__(self, play_tune_fn) -> None:
        super().__init__(play_tune_fn, TUNE_SCAN_ACTIVE, SCAN_BEEP_INTERVAL_S)


class PostflightBeeper(PeriodicBeeper):
    def __init__(self, play_tune_fn) -> None:
        super().__init__(play_tune_fn, TUNE_POSTPROCESS_ACTIVE, POSTPROCESS_BEEP_INTERVAL_S)


class FaultAlarmManager:
    def __init__(self, play_tune_fn, is_armed_fn, startup_failure_repeats: bool = True) -> None:
        self._play = play_tune_fn
        self._is_armed = is_armed_fn
        self._startup_failure_repeats = startup_failure_repeats

        self._warning_latched = False
        self._error_active = False
        self._critical_active = False
        self._failure_active = False

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)

    def set_warning(self, active: bool = True) -> None:
        if active and not self._warning_latched:
            self._warning_latched = True
            try:
                self._play(TUNE_WARNING)
            except Exception:
                pass
        elif not active:
            self._warning_latched = False

    def set_error(self, active: bool) -> None:
        self._error_active = bool(active)

    def set_critical(self, active: bool) -> None:
        self._critical_active = bool(active)

    def set_system_failure(self, active: bool) -> None:
        self._failure_active = bool(active)

    def clear_all(self) -> None:
        self._warning_latched = False
        self._error_active = False
        self._critical_active = False
        self._failure_active = False

    def _current_alarm(self):
        if self._failure_active:
            if self._is_armed() or self._startup_failure_repeats:
                return TUNE_SYSTEM_FAILURE, SYSTEM_FAILURE_INTERVAL_S
        if not self._is_armed():
            return None, 0.1
        if self._critical_active:
            return TUNE_CRITICAL, CRITICAL_REPEAT_INTERVAL_S
        if self._error_active:
            return TUNE_ERROR, ERROR_REPEAT_INTERVAL_S
        return None, 0.1

    def _loop(self) -> None:
        while self._running:
            tune, interval_s = self._current_alarm()
            if tune is None:
                time.sleep(interval_s)
                continue
            try:
                self._play(tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._running, interval_s)


class BenchSoundLedTester(Node):
    def __init__(self) -> None:
        super().__init__("bench_sound_led_tester")
        self._tune_pub = self.create_publisher(PlayTuneV2, "/mavros/play_tune", 10)

        self.armed = False
        self.fcu = True
        self.processing = False
        self.stack_running = False

        self.transient_led_state = ""
        self.transient_led_until = 0.0

        self.scan_beeper = ScanBeeper(self.play_tune)
        self.post_beeper = PostflightBeeper(self.play_tune)
        self.alarms = FaultAlarmManager(
            play_tune_fn=self.play_tune,
            is_armed_fn=lambda: self.armed,
            startup_failure_repeats=True,
        )

        self._status_running = True
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()

    def play_tune(self, tune: str) -> None:
        msg = PlayTuneV2()
        msg.format = TUNE_FORMAT
        msg.tune = tune
        self._tune_pub.publish(msg)

    def set_led_event(self, state: str, duration_s: float) -> None:
        self.transient_led_state = state
        self.transient_led_until = time.time() + duration_s

    def clear_led_event(self) -> None:
        self.transient_led_state = ""
        self.transient_led_until = 0.0

    def stop_all_repeating(self) -> None:
        self.scan_beeper.stop()
        self.post_beeper.stop()
        self.processing = False
        self.stack_running = False
        self.alarms.clear_all()

    def shutdown_clean(self) -> None:
        self.stop_all_repeating()
        self._status_running = False
        self._status_thread.join(timeout=1.0)
        self.alarms.shutdown()
        try:
            Path(WATCHDOG_STATUS_FILE).unlink(missing_ok=True)
            Path(WATCHDOG_STATUS_FILE_TMP).unlink(missing_ok=True)
        except Exception:
            pass

    def _write_status(self) -> None:
        payload = {
            "ts": time.time(),
            "fcu": self.fcu,
            "armed": self.armed,
            "processing": self.processing,
            "stack_running": self.stack_running,
            "led_state": self.transient_led_state,
            "led_until": self.transient_led_until,
            "warning": self.alarms._warning_latched,
            "error": self.alarms._error_active,
            "critical": self.alarms._critical_active,
            "system_failure": self.alarms._failure_active,
        }
        try:
            with open(WATCHDOG_STATUS_FILE_TMP, "w") as f:
                json.dump(payload, f)
            Path(WATCHDOG_STATUS_FILE_TMP).replace(WATCHDOG_STATUS_FILE)
        except Exception:
            pass

    def _status_loop(self) -> None:
        while self._status_running:
            if self.transient_led_until and time.time() >= self.transient_led_until:
                self.clear_led_event()
            self._write_status()
            time.sleep(0.1)


def print_menu(t: BenchSoundLedTester) -> None:
    print("\n" + "=" * 76)
    print(" DronePi Bench Sound + LED Test")
    print("=" * 76)
    print(f" Armed: {'ON' if t.armed else 'OFF'} | FCU: {'ON' if t.fcu else 'OFF'}")
    print(" Event tones + LED events:")
    print("   1  System start")
    print("   2  Scan ready")
    print("   3  Scan start")
    print("   4  Scan finished")
    print("   5  Post-process done")
    print(" Repeating non-fault states:")
    print("   6  Toggle scan-active repeater + LED")
    print("   7  Toggle post-process repeater + LED")
    print(" Faults:")
    print("   8  Warning (one-shot + LED)")
    print("   9  Toggle ERROR latch + LED")
    print("  10  Toggle CRITICAL latch + LED")
    print("  11  Toggle SYSTEM FAILURE latch + LED")
    print(" Controls:")
    print("  12  Toggle local armed flag")
    print("  13  Toggle fake FCU connected")
    print("  14  Clear all fault latches")
    print("  15  Stop all repeating sounds/states")
    print("  16  Run full bench demo")
    print("   q  Quit")
    print("=" * 76)
    print(f" Faults: error={t.alarms._error_active} critical={t.alarms._critical_active} failure={t.alarms._failure_active}")
    print(f" Repeaters: scan={t.scan_beeper.active} post={t.post_beeper.active}")


def run_demo(t: BenchSoundLedTester) -> None:
    print("\n[DEMO] Starting sound+LED demo...")
    t.stop_all_repeating()
    t.fcu = True

    print("[DEMO] System start")
    t.set_led_event("SYSTEM_START", 2.2)
    t.play_tune(TUNE_SYSTEM_START)
    time.sleep(2.4)

    print("[DEMO] Scan ready")
    t.set_led_event("SCAN_READY", 1.0)
    t.play_tune(TUNE_SCAN_READY)
    time.sleep(1.2)

    print("[DEMO] Scan start")
    t.set_led_event("SCAN_START", 1.2)
    t.play_tune(TUNE_SCAN_START)
    time.sleep(1.3)

    print("[DEMO] Fake scanning active")
    t.stack_running = True
    t.scan_beeper.start()
    time.sleep(4.0)
    t.scan_beeper.stop()
    t.stack_running = False

    print("[DEMO] Scan finished")
    t.set_led_event("SCAN_FINISHED", 1.5)
    t.play_tune(TUNE_SCAN_FINISHED)
    time.sleep(1.7)

    print("[DEMO] Fake post-processing")
    t.processing = True
    t.post_beeper.start()
    time.sleep(4.0)
    t.post_beeper.stop()
    t.processing = False

    print("[DEMO] Post-process done")
    t.play_tune(TUNE_POSTPROCESS_DONE)
    time.sleep(1.5)

    print("[DEMO] Warning")
    t.alarms.set_warning(True)
    time.sleep(1.5)
    t.alarms.set_warning(False)

    print("[DEMO] Error repeating")
    t.armed = True
    t.alarms.set_error(True)
    time.sleep(5.0)
    t.alarms.set_error(False)
    time.sleep(0.5)

    print("[DEMO] Critical repeating")
    t.alarms.set_critical(True)
    time.sleep(5.0)
    t.alarms.set_critical(False)
    time.sleep(0.5)

    print("[DEMO] System failure repeating")
    t.alarms.set_system_failure(True)
    time.sleep(5.0)
    t.alarms.set_system_failure(False)
    t.armed = False
    print("[DEMO] Demo complete.\n")


def main() -> None:
    rclpy.init()
    tester = BenchSoundLedTester()
    time.sleep(0.5)

    try:
        while True:
            print_menu(tester)
            choice = input("\nSelect > ").strip().lower()

            if choice == "q":
                break
            elif choice == "1":
                tester.set_led_event("SYSTEM_START", 2.2)
                tester.play_tune(TUNE_SYSTEM_START)
            elif choice == "2":
                tester.set_led_event("SCAN_READY", 1.0)
                tester.play_tune(TUNE_SCAN_READY)
            elif choice == "3":
                tester.set_led_event("SCAN_START", 1.2)
                tester.play_tune(TUNE_SCAN_START)
            elif choice == "4":
                tester.set_led_event("SCAN_FINISHED", 1.5)
                tester.play_tune(TUNE_SCAN_FINISHED)
            elif choice == "5":
                tester.play_tune(TUNE_POSTPROCESS_DONE)
            elif choice == "6":
                if tester.scan_beeper.active:
                    tester.scan_beeper.stop()
                    tester.stack_running = False
                    print("[INFO] Scan repeater stopped")
                else:
                    tester.stack_running = True
                    tester.scan_beeper.start()
                    print("[INFO] Scan repeater started")
            elif choice == "7":
                if tester.post_beeper.active:
                    tester.post_beeper.stop()
                    tester.processing = False
                    print("[INFO] Post-process repeater stopped")
                else:
                    tester.processing = True
                    tester.post_beeper.start()
                    print("[INFO] Post-process repeater started")
            elif choice == "8":
                tester.alarms.set_warning(True)
                print("[INFO] Warning played once")
            elif choice == "9":
                tester.alarms.set_error(not tester.alarms._error_active)
                print(f"[INFO] ERROR latch -> {tester.alarms._error_active}")
            elif choice == "10":
                tester.alarms.set_critical(not tester.alarms._critical_active)
                print(f"[INFO] CRITICAL latch -> {tester.alarms._critical_active}")
            elif choice == "11":
                tester.alarms.set_system_failure(not tester.alarms._failure_active)
                print(f"[INFO] SYSTEM FAILURE latch -> {tester.alarms._failure_active}")
            elif choice == "12":
                tester.armed = not tester.armed
                print(f"[INFO] Armed flag -> {tester.armed}")
            elif choice == "13":
                tester.fcu = not tester.fcu
                print(f"[INFO] Fake FCU -> {tester.fcu}")
            elif choice == "14":
                tester.alarms.clear_all()
                print("[INFO] Cleared all fault latches")
            elif choice == "15":
                tester.stop_all_repeating()
                tester.clear_led_event()
                print("[INFO] Stopped all repeaters/states")
            elif choice == "16":
                run_demo(tester)
            else:
                print("[WARN] Invalid selection")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        tester.shutdown_clean()
        tester.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
