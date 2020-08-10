#!/bin/bash
#This script is used to create directories for static_maps for better organization

# Create directory in 2d folder named the argument passed to the script
cd ~/catkin_ws/src/uml_3d_race/resources/static_maps/2d
mkdir -p $1

# Create directory in 3d folder named the argument passed to the script
cd ~/catkin_ws/src/uml_3d_race/resources/static_maps/3d
mkdir -p $1

exit 0
