import os
import toml

# Conveniences to other module directories via relative paths
ROBORACER_ASSETS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
"""Path to the extension source directory."""

ROBORACER_ASSETS_DATA_DIR = os.path.join(ROBORACER_ASSETS_EXT_DIR, "data")
"""Path to the extension data directory."""