"""Waypoint execution and mission state machine.

Loads mission plans from YAML templates and executes them
as a sequence of waypoints with the PX4 interface.
"""

import yaml
import time
import numpy as np
from enum import Enum
from pathlib import Path
from typing import Optional, List

from utils.logger import setup_logger

logger = setup_logger(__name__)


class MissionState(Enum):
    IDLE = "idle"
    PRE_FLIGHT = "pre_flight"
    TAKING_OFF = "taking_off"
    EXECUTING = "executing"
    RETURNING = "returning"
    LANDING = "landing"
    COMPLETE = "complete"
    ABORTED = "aborted"


class MissionExecutor:
    """Executes a sequence of waypoints using the PX4 interface."""

    def __init__(self, px4_interface, config: dict):
        """Initialize mission executor.

        Args:
            px4_interface: PX4Interface node instance.
            config: Flight config dict.
        """
        self.px4 = px4_interface
        self.cruise_speed = config.get("flight", {}).get("cruise_speed", 2.0)
        self.takeoff_altitude = config.get("flight", {}).get("takeoff_altitude", 5.0)

        self.state = MissionState.IDLE
        self.waypoints: List[List[float]] = []
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.5  # meters — close enough to waypoint

        # Callbacks
        self._on_waypoint_reached = None
        self._on_state_change = None

    def load_mission(self, mission_file: str) -> bool:
        """Load a mission from a YAML file.

        Args:
            mission_file: Path to mission YAML file.

        Returns:
            True if loaded successfully.
        """
        try:
            with open(mission_file, "r") as f:
                mission = yaml.safe_load(f)

            mission_data = mission.get("mission", {})
            mission_type = mission_data.get("type", "unknown")

            if mission_type == "survey":
                self.waypoints = self._generate_grid_waypoints(mission_data)
            elif mission_type == "perimeter":
                self.waypoints = mission_data.get("waypoints", [])
            else:
                logger.error(f"Unknown mission type: {mission_type}")
                return False

            logger.info(
                f"Loaded mission '{mission_data.get('name', 'unnamed')}' "
                f"with {len(self.waypoints)} waypoints"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            return False

    def _generate_grid_waypoints(self, mission_data: dict) -> List[List[float]]:
        """Generate grid survey waypoints from mission parameters."""
        area = mission_data.get("area", {})
        grid = mission_data.get("grid", {})

        length = area.get("length", 20.0)
        width = area.get("width", 15.0)
        altitude = -area.get("altitude", 5.0)  # NED: negative is up
        spacing = grid.get("line_spacing", 3.0)

        waypoints = []
        y = 0.0
        forward = True

        while y <= width:
            if forward:
                waypoints.append([0.0, y, altitude])
                waypoints.append([length, y, altitude])
            else:
                waypoints.append([length, y, altitude])
                waypoints.append([0.0, y, altitude])
            forward = not forward
            y += spacing

        return waypoints

    def set_waypoint_callback(self, callback):
        """Register callback for when a waypoint is reached."""
        self._on_waypoint_reached = callback

    def set_state_callback(self, callback):
        """Register callback for state changes."""
        self._on_state_change = callback

    def _set_state(self, new_state: MissionState):
        old_state = self.state
        self.state = new_state
        logger.info(f"Mission state: {old_state.value} -> {new_state.value}")
        if self._on_state_change:
            self._on_state_change(old_state, new_state)

    def start(self) -> bool:
        """Begin mission execution.

        Returns:
            True if mission started successfully.
        """
        if not self.waypoints:
            logger.error("No waypoints loaded")
            return False

        self._set_state(MissionState.PRE_FLIGHT)
        self.current_waypoint_idx = 0
        return True

    def update(self) -> MissionState:
        """Call this periodically to advance the mission state machine.

        Returns:
            Current mission state.
        """
        if self.state == MissionState.EXECUTING:
            if self.current_waypoint_idx >= len(self.waypoints):
                self._set_state(MissionState.RETURNING)
                self.px4.return_to_launch()
                return self.state

            # Send current waypoint
            wp = self.waypoints[self.current_waypoint_idx]
            self.px4.goto_position(wp[0], wp[1], wp[2])

            # Check if reached
            pos = np.array(self.px4.position)
            target = np.array(wp)
            dist = np.linalg.norm(pos - target)

            if dist < self.waypoint_threshold:
                logger.info(
                    f"Reached waypoint {self.current_waypoint_idx + 1}"
                    f"/{len(self.waypoints)}"
                )
                if self._on_waypoint_reached:
                    self._on_waypoint_reached(self.current_waypoint_idx, wp)
                self.current_waypoint_idx += 1

        return self.state

    def abort(self):
        """Abort the mission and return to launch."""
        logger.warning("Mission aborted!")
        self._set_state(MissionState.ABORTED)
        self.px4.return_to_launch()

    @property
    def progress(self) -> float:
        """Mission completion percentage (0-100)."""
        if not self.waypoints:
            return 0.0
        return (self.current_waypoint_idx / len(self.waypoints)) * 100.0
