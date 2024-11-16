#!/bin/bash
echo "Starting build script for mashengl"
cd ~/catkin_ws
rm -rf src/ece642rtle
tar -zxvf ~/mashengl_files.tgz -C src/

cd ~/catkin_ws
catkin_make clean
catkin_make
catkin_make ece642rtle_turn_monitor
source devel/setup.bash
echo "Build script completed successfully."
