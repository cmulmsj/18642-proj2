#!/bin/bash

# Function to kill all processes
kill_processes() {
    # First kill the monitor properly
    MONITORS="ece642rtle_turn_monitor"
    if [[ -n $(pgrep -f "${MONITORS:0:15}") ]]; then
        echo "Stopping monitor..."
        kill $(pgrep -f "${MONITORS:0:15}")
        sleep 2
    fi

    # Process violations
    if [ -s "${MONITORS}.output.tmp" ]; then
        echo "" >> VIOLATIONS.txt
        echo "Monitor $MONITORS Violations:" >> VIOLATIONS.txt
        echo "" >> VIOLATIONS.txt
        grep -C 5 "[ WARN]" ${MONITORS}.output.tmp >> VIOLATIONS.txt
        VIOL_COUNT=`grep "[ WARN]" ${MONITORS}.output.tmp | wc -l`
        echo "TOTAL VIOLATIONS: $VIOL_COUNT"
        rm ${MONITORS}.output.tmp
        echo "" >> VIOLATIONS.txt
    fi

    # Then kill other processes
    for p in "$@"; do
        if [[ -z $(ps -p $p > /dev/null) ]]; then
            echo "Killing process $p"
            kill $p
            sleep 1
        fi
    done

    echo "All processes terminated. Check VIOLATIONS.txt for details."
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

# Have to kill BG process if user exits
trap 'kill_processes $ROSCORE_PID' SIGINT
sleep 5

# Set maze file parameter
rosparam set /maze_file "$maze_file"

# Start monitor
echo "Starting monitor..."
cd "$turtledir/monitors"
# Clean up any existing files
rm -f VIOLATIONS.txt
rm -f ece642rtle_turn_monitor.output.tmp
stdbuf -oL ./run_642_monitors.sh ece642rtle_turn_monitor &
sleep 5

# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1

if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node"
    kill_processes $ROSCORE_PID
    exit 1
fi

trap 'kill_processes $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!

trap 'kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

echo "Running turtle. Will auto-terminate after reaching goal..."

# Monitor for goal state
GOAL_COUNT=0
while [ 1 -eq 1 ]; do
    if grep -q "At End Request.*resp = true" "$turtledir/monitors/ece642rtle_turn_monitor.output.tmp" 2>/dev/null; then
        GOAL_COUNT=$((GOAL_COUNT + 1))
        if [ $GOAL_COUNT -ge 10 ]; then
            echo "Goal reached. Initiating shutdown..."
            kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID
        fi
    else
        GOAL_COUNT=0
    fi
    sleep 1
done
