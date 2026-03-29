"""flight/ — DronePi flight stack components.

Architecture
------------
This package contains two categories of modules that serve different
callers and should not be confused with each other:

RUNTIME MODULES  (started by systemd or main.py at boot)
---------------------------------------------------------
main.py (project root)
    Top-level state machine. Implements its own MainNode class internally.
    Does NOT import FlightController. Manages MODE 1/2/3 logic and
    coordinates with drone_watchdog.py via /tmp/dronepi_mission.lock.

drone_watchdog.py
    Subprocess supervisor. Starts/stops Point-LIO, SLAM bridge, and
    bag recorder. Reads the lock file to yield to main.py in MODE 3.
    Does NOT import FlightController.

_slam_bridge.py
    ROS 2 node. Forwards Point-LIO odometry to MAVROS vision_pose topic
    for EKF2 fusion. Launched as a subprocess by main.py and watchdog.

collision_monitor.py
    ROS 2 node. Publishes obstacle distance and AGL height from LiDAR.
    Launched as a subprocess by main.py in MODE 3.

gap_detector.py
    LiDAR coverage gap detector. Imported directly by main.py during
    MODE 3 autonomous waypoint execution for real-time gap fill.

TEST UTILITY MODULES  (run manually, never started by systemd)
--------------------------------------------------------------
flight_controller.py
    Reusable MAVROS primitives: arm, set_mode, fly_to, EKF wait.
    Used ONLY by standalone test scripts such as test_offboard_flight.py.
    NOT used by main.py.

watchdog_core/
    Subpackage containing the internal modules of drone_watchdog.py:
    buzzer.py, mavros_reader.py, flight_stack.py, postflight.py,
    logging_utils.py.

Public export
-------------
FlightController is exported for test script convenience only.
Import it as:

    from flight.flight_controller import FlightController

or, from a test script in the tests/ directory:

    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'flight'))
    from flight_controller import FlightController
"""

from .flight_controller import FlightController, create_controller

__all__ = [
    "FlightController",
    "create_controller",
]

__version__ = "2.1.0"
__author__  = "DronePi Capstone Team"
