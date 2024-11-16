#!/bin/bash
# Function to kill all processes
kill_processes() {
    for p in "$@"; do
        if [[ -z $(ps -p $p > /dev/null) ]]; then
            echo "Killing process $p"
            kill $p
            sleep 1
        fi
    done
    echo "killed all processes, exiting"
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
# Start monitor (added this section)
cd "$turtledir/monitors"
./run_642_monitors.sh ece642rtle_turn_monitor &
sleep 5
    if [ -s "$m.output.tmp" ]; then
        echo "" >> VIOLATIONS.txt
        echo "Monitor $m Violations:" >> VIOLATIONS.txt
        echo "" >> VIOLATIONS.txt
        grep -C 5 "\[ WARN\]" $m.output.tmp >> VIOLATIONS.txt
        m_viol=`grep "\[ WARN\]" $m.output.tmp | wc -l`
        total_viol=$(( total_viol + m_viol))
        rm $m.output.tmp
        echo "" >> VIOLATIONS.txt
    fi
# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1
if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node"
    kill_processes $ROSCORE_PID
    exit 1
fi
# Have to kill BG processes if user exits
trap 'kill_processes $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9
# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!
# Have to kill BG processes if user exits
trap 'kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT
# Wait for Ctrl+C
while [ 1 -eq 1 ]; do
    sleep 30
done
# Return to home directory
cd ~
