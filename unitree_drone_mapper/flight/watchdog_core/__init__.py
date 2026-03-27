"""watchdog_core — Modular components for the DronePi flight stack watchdog.

Components
----------
  MavrosReader       ROS 2 interface: FCU state, RC input, buzzer output
  FlightStack        Point-LIO / SLAM bridge / bag recorder process manager
  PostflightMonitor  Post-flight subprocess with log piping and beeper feedback

See individual module docstrings for detailed documentation.
"""

from .mavros_reader import MavrosReader
from .flight_stack  import FlightStack
from .postflight    import PostflightMonitor

__all__ = [
    "MavrosReader",
    "FlightStack",
    "PostflightMonitor",
]

__version__ = "2.0.0"
__author__  = "DronePi Capstone Team"
