#!/bin/bash

# Function to kill all processes and check violations
kill_processes() {
    echo "Shutting down and checking for violations..."
    
    # Kill the monitor first and wait for VIOLATIONS.txt
    if [[ -n "$MONITOR_PID" ]]; then
        kill $MONITOR_PID
        # Wait a moment for VIOLATIONS.txt to be generated
        sleep 2
        if [ -f "$turtledir/monitors/VIOLATIONS.txt" ]; then
            echo "=================== VIOLATIONS FOUND ==================="
            cat "$turtledir/monitors/VIOLATIONS.txt"
            echo "===================================================="
        else
            echo "No violations detected"
        fi
    fi

    # Kill other processes
    for p in "$@"; do
        if [[ -z $(ps -p $p > /dev/null) ]]; then
            echo "Killing process $p"
            kill $p
            sleep 1
        fi
    done
    echo "All processes terminated"
    exit 0
}

# Check if maze number is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <maze_number>"
    exit 1
fi

MAZE_NUM=$1
maze_file="m${MAZE_NUM}.maze"

# Source the workspace setup
source ~/catkin_ws/devel/setup.bash

# Locate the ece642rtle directory
turtledir=`rospack find ece642rtle`
if [ -z "$turtledir" ]; then
    echo "Cannot locate package ece642rtle."
    exit 1
fi

# Check that maze file exists
if [ ! -s "$turtledir/$maze_file" ]; then
    echo "Maze file $maze_file does not exist."
    exit 1
fi

# Run roscore if it is not already running
ROSCORE_PID=""
if [[ -z $(pgrep roscore) ]]; then
    roscore&
    ROSCORE_PID=$!
    sleep 1
    if [[ -z $(pgrep roscore) ]]; then
        echo "Error launching roscore. Make sure no other ros processes are running and try again."
        exit 1
    fi
fi

# Start with just roscore in trap
trap 'kill_processes $ROSCORE_PID' SIGINT
sleep 5

# Set maze file parameter
rosparam set /maze_file "$maze_file"

# Start turn monitor
echo "Starting turn monitor..."
cd $turtledir/monitors
./run_642_monitors.sh ece642rtle_turn_monitor &
MONITOR_PID=$!
sleep 2

# Add monitor to trap
trap 'kill_processes $MONITOR_PID $ROSCORE_PID' SIGINT

# Node that displays the maze and runs the turtle
cd $turtledir
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1

if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node"
    kill_processes $MONITOR_PID $ROSCORE_PID
    exit 1
fi

# Update trap with turtle
trap 'kill_processes $MONITOR_PID $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!

# Final trap with all processes
trap 'kill_processes $MONITOR_PID $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

echo "All processes started. Monitor active. Press Ctrl+C to stop and view violations."

# Wait for Ctrl+C
while [ 1 -eq 1 ]; do
    sleep 30
done
