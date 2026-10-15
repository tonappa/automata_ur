#!/bin/bash
set -e

# source base ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

WORKSPACE_PATH="/home/ros/${WORKSPACE_NAME}"

# source workspace if already compiled
if [ "$ROS_VERSION" = "1" ] && [ -f "${WORKSPACE_PATH}/devel/setup.bash" ]; then
    echo "Sourcing ROS 1 workspace: ${WORKSPACE_PATH}/devel/setup.bash"
    source "${WORKSPACE_PATH}/devel/setup.bash"
elif [ "$ROS_VERSION" = "2" ] && [ -f "${WORKSPACE_PATH}/install/setup.bash" ]; then
    echo "Sourcing ROS 2 workspace: ${WORKSPACE_PATH}/install/setup.bash"
    source "${WORKSPACE_PATH}/install/setup.bash"
else
    echo "Workspace not built yet. Run: colcon build --cmake-args -DCMAKE_CXX_FLAGS=\"-w\" -DBUILD_TESTING=OFF --executor sequential"
fi

exec "$@"
