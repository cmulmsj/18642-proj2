#!/bin/bash

# Function to kill all processes and process monitor output
kill_processes() {
    echo "Shutting down..."
    
    # First, stop the monitor gracefully
    if [[ -n $(pgrep -f "ece642rtle_turn_monitor") ]]; then
        echo "Stopping monitor..."
        cd "$turtledir/monitors"
        pkill -SIGINT -f "run_642_monitors"
        sleep 3  # Give monitor time to process final messages
    fi
    
    # Stop ROS nodes gracefully
    if [[ -n $(pgrep -f "ece642rtle_student") ]]; then
        echo "Stopping student node..."
        rosnode kill /ece642rtle_student >/dev/null 2>&1
        sleep 1
    fi
    
    if [[ -n $(pgrep -f "ece642rtle_node") ]]; then
        echo "Stopping turtle node..."
        rosnode kill /ece642rtle_node >/dev/null 2>&1
        sleep 1
    fi
    
    # Process monitor output
    cd "$turtledir/monitors"
    if [ -f "ece642rtle_turn_monitor.output.tmp" ]; then
        echo "Processing monitor output..."
        touch VIOLATIONS.txt  # Ensure file exists
        grep -C 5 "[ WARN]" ece642rtle_turn_monitor.output.tmp >> VIOLATIONS.txt 2>/dev/null
        VIOL_COUNT=$(grep -c "[ WARN]" ece642rtle_turn_monitor.output.tmp 2>/dev/null || echo "0")
        echo "TOTAL VIOLATIONS: $VIOL_COUNT"
        rm -f ece642rtle_turn_monitor.output.tmp
    fi

    # Finally cleanup any remaining processes
    for p in "$@"; do
        if kill -0 $p 2>/dev/null; then
            kill $p 2>/dev/null || true
        fi
    done
    
    # Shutdown roscore last
    if [[ -n "$ROSCORE_PID" ]]; then
        echo "Stopping roscore..."
        kill $ROSCORE_PID 2>/dev/null || true
        sleep 1
    fi
    
    echo "All processes terminated"
    exit 0
}

# Function to check if turtle is at goal
check_goal_state() {
    local goal_count=0
    local max_goal_cycles=10

    while true; do
        if grep -q "At End Request.*resp = true" "$turtledir/monitors/ece642rtle_turn_monitor.output.tmp" 2>/dev/null; then
            goal_count=$((goal_count + 1))
            if [ $goal_count -ge $max_goal_cycles ]; then
                echo "Turtle maintained goal position for $max_goal_cycles cycles. Shutting down..."
                kill_processes $STUDENT_PID $MONITOR_PID $TURTLE_PID $ROSCORE_PID
            fi
        fi
        sleep 1
    done
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

# Start monitor with longer sleep
echo "Starting turn monitor..."
cd "$turtledir/monitors"
if [ -f "VIOLATIONS.txt" ]; then
    rm VIOLATIONS.txt
fi
./run_642_monitors.sh ece642rtle_turn_monitor &
MONITOR_PID=$!
sleep 5  # Increased sleep time for monitor startup

# Add monitor to trap
trap 'kill_processes $MONITOR_PID $ROSCORE_PID' SIGINT

# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 2  # Increased sleep time

if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node"
    kill_processes $MONITOR_PID $ROSCORE_PID
    exit 1
fi

# Update trap with turtle
trap 'kill_processes $MONITOR_PID $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 5  # Additional sleep before student node

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!

# Final trap with all processes
trap 'kill_processes $STUDENT_PID $MONITOR_PID $TURTLE_PID $ROSCORE_PID' SIGINT

echo "All processes started. Will auto-terminate after goal is reached..."

# Start goal state checking in background
check_goal_state &
GOAL_CHECK_PID=$!

# Wait for either goal achievement or Ctrl+C
wait $GOAL_CHECK_PID
