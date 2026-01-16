"""
Blender to Stonefish Converter Plugin

This package provides tools to convert Blender scenes to Stonefish scenario files.
"""

import logging
import sys

# Package version
__version__ = "1.0.0"


# Configure logging for the entire package
def setup_logging(level=logging.INFO, log_file=None):
    """
    Configure logging for the blender_stonefish_plugin package.

    Args:
        level: Logging level (default: logging.INFO)
        log_file: Optional file path to write logs to
    """
    # Create logger
    logger = logging.getLogger("blender_stonefish_plugin")
    logger.setLevel(level)

    # Remove existing handlers
    logger.handlers.clear()

    # Create formatter
    formatter = logging.Formatter(
        fmt="[%(asctime)s] %(levelname)-8s %(name)s - %(message)s", datefmt="%H:%M:%S"
    )

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler (optional)
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


# Initialize logging with default settings
setup_logging()

# Expose main classes
from .blender_extractor import BlenderExtractor, BlenderObject
from .main import ConfigLoader, StonefishScenarioBuilder

__all__ = [
    "BlenderExtractor",
    "BlenderObject",
    "ConfigLoader",
    "StonefishScenarioBuilder",
    "setup_logging",
]
