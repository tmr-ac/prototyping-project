#!/bin/bash
source /opt/ros/humble/setup.bash
cd
cd ros2_ws/
. install/setup.bash
colcon build --packages-select bin_picking_project
