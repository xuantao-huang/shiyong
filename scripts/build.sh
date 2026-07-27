#!/bin/bash
# YOPO Unified Workspace Build Script
# Builds Livox-SDK2 (standalone cmake) then the full catkin workspace.
#
# Usage:
#   cd /home/nv/YOPO-C5
#   bash scripts/build.sh
#
# On first run, Livox-SDK2 will be installed system-wide (requires sudo).
# On subsequent runs, pass --skip-sdk to skip the SDK step.

set -e
YOPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$YOPO_DIR/ros_ws"
NPROC=$(nproc)

SKIP_SDK=0
for arg in "$@"; do
    [[ "$arg" == "--skip-sdk" ]] && SKIP_SDK=1
done

echo "================================================================"
echo " YOPO Workspace: $YOPO_DIR"
echo " Jobs: $NPROC"
echo "================================================================"

# ── Step 1: Livox-SDK2 (standalone cmake, must be installed before catkin) ──
if [[ $SKIP_SDK -eq 0 ]]; then
    echo ""
    echo ">>> [1/2] Building Livox-SDK2..."
    SDK_DIR="$ROS_WS/src/Livox-SDK2"
    mkdir -p "$SDK_DIR/build"
    cd "$SDK_DIR/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j"$NPROC"
    sudo make install
    sudo ldconfig
    echo ">>> Livox-SDK2 installed."
else
    echo ">>> [1/2] Skipping Livox-SDK2 (--skip-sdk)."
fi

# ── Step 2: catkin workspace ──
echo ""
echo ">>> [2/2] Building catkin workspace..."
source /opt/ros/noetic/setup.bash
cd "$ROS_WS"
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -j"$NPROC"

echo ""
echo "================================================================"
echo " Build complete!"
echo " To use the workspace, add to your shell or let start.sh handle it:"
echo "   source $ROS_WS/devel/setup.bash"
echo "================================================================"
