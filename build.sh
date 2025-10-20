#!/bin/bash
# # Portable build script for ROS2 packages with virtual environment
# # This script works from any location and uses relative paths
#
# set -e  # Exit on error
#
# SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# REPO_ROOT="$SCRIPT_DIR"
# ROS2_WS="$REPO_ROOT/ros2_ws"
# VENV_DIR="$REPO_ROOT/.venv"
#
# # Detect Python version dynamically
# PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
#
# echo "📁 Repository root: $REPO_ROOT"
# echo "🐍 Python version: $PYTHON_VERSION"
# echo ""
#
# # Check if virtual environment exists
# if [ ! -d "$VENV_DIR" ]; then
#     echo "❌ Virtual environment not found at $VENV_DIR"
#     echo "Please create it first: python3 -m venv .venv"
#     exit 1
# fi
#
# cd "$ROS2_WS"
#
# source "$VENV_DIR/bin/activate"
# source /opt/ros/jazzy/setup.bash
#
# # Building packages
# colcon build --symlink-install \
#     --cmake-args \
#     -DPython3_EXECUTABLE="$VENV_DIR/bin/python3" \
#     -DPython3_NumPy_INCLUDE_DIR="$VENV_DIR/lib/python${PYTHON_VERSION}/site-packages/numpy/core/include"
#
# if [ $? -eq 0 ]; then
#     echo ""
#     echo "🔧 Fixing shebangs to use virtual environment Python..."
#     # Fix shebangs in all Python scripts in install/*/lib/*/
    find install/*/lib/*/ -type f -executable 2>/dev/null | while read script; do
        if head -n1 "$script" | grep -q "^#!/usr/bin/python"; then
            # Use the venv Python path
            VENV_PYTHON="$VENV_DIR/bin/python3"
            # Replace the shebang
            sed -i "1s|^#!/usr/bin/python.*|#!$VENV_PYTHON|" "$script"
            echo "  Fixed: $script"
        fi
    done
#     echo ""
#     echo "✅ Build successful!"
# else
#     echo "❌ Build failed!"
#     exit 1
# fi
