"""Unified logging configuration for the drone mapping system."""

import logging
import os
from datetime import datetime
from pathlib import Path


def setup_logger(
    name: str,
    level: str = "INFO",
    log_dir: str | None = None,
    session_id: str | None = None,
) -> logging.Logger:
    """Create a configured logger with console and optional file output.

    Args:
        name: Logger name (typically module name).
        level: Log level string (DEBUG, INFO, WARNING, ERROR).
        log_dir: Directory to write log files. None = console only.
        session_id: Flight session ID for log file naming.

    Returns:
        Configured logger instance.
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))

    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        fmt="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Console handler
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)

    # File handler (if log directory specified)
    if log_dir:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        suffix = f"_{session_id}" if session_id else ""
        log_file = os.path.join(log_dir, f"{name}{suffix}.log")
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def get_session_id() -> str:
    """Generate a session ID from the current timestamp."""
    return datetime.now().strftime("%Y%m%d_%H%M%S")
