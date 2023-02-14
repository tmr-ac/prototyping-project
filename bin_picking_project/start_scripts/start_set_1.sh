#!/bin/bash
source /opt/ros/humble/setup.bash
cd
cd ros2_ws/
. install/setup.bash
ros2 launch bin_picking_project launch1.py
