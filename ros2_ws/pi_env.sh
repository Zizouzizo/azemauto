#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/src/my_robot_bringup/config/network/cyclonedds.xml"

echo "azemauto ROS 2 environment loaded for Raspberry Pi"
