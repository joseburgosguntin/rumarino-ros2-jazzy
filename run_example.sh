#!/bin/bash
# Exit on error
set -e


# Check for required Python libraries
echo "Checking required Python libraries..."
MISSING_LIBS=0

if ! python3 -c "import yaml" 2>/dev/null; then
    echo "Error: PyYAML is not installed."
    echo "Please install it with: pip install pyyaml"
    MISSING_LIBS=1
fi

if ! python3 -c "import inotify" 2>/dev/null; then
    echo "Error: inotify is not installed."
    echo "Please install it with: pip install inotify"
    MISSING_LIBS=1
fi

if [ $MISSING_LIBS -eq 1 ]; then
    echo ""
    echo "Missing required libraries. Please install them and try again."
    exit 1
fi

WORKSPACE_ROOT="$(pwd)"
PLUGIN_DIR="$WORKSPACE_ROOT/blender_stonefish_plugin"
BLEND_FILE="$PLUGIN_DIR/example.blend"
CONFIG_FILE="$PLUGIN_DIR/config.yaml"
OUTPUT_SCN="$WORKSPACE_ROOT/src/controller_stonefish/data/scenarios/generated.scn"

echo "Converting example.blend to Stonefish scenario..."
cd "$PLUGIN_DIR"
python3 main.py "$BLEND_FILE" -o "$OUTPUT_SCN" -c "$CONFIG_FILE"
echo ""

echo "Building controller_stonefish package..."
cd "$WORKSPACE_ROOT"
colcon build --packages-select controller_stonefish --symlink-install
echo ""

echo "Launching Stonefish simulator..."
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
ros2 launch bringup test_mission_executor.launch.py mission_name:=prequalify env_file_name:="$OUTPUT_SCN"
