#!/bin/bash

# Exit on any error
set -e

# Check if maze number is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <maze_number>"
    exit 1
fi

MAZE_NUM=$1

# Source the workspace setup
source ~/catkin_ws/devel/setup.bash

# Kill any existing ROS nodes and clean up
echo "Cleaning up any existing ROS processes..."
killall -9 roscore rosmaster rosout ece642rtle_student 2>/dev/null || true
sleep 2

# Start roscore in the background
echo "Starting ROS core..."
roscore &
sleep 3

# Launch the maze with specified number
echo "Launching maze $MAZE_NUM..."
rosrun ece642rtle ece642rtle_student _maze_file:=$(rospack find ece642rtle)/m${MAZE_NUM}.maze

# Wait for Ctrl+C
echo "Press Ctrl+C to exit..."
wait

# Cleanup after ctrl+c
echo "Cleaning up..."
killall -9 roscore rosmaster rosout ece642rtle_student 2>/dev/null || true

# Return to home directory
cd ~

echo "Run completed!"
