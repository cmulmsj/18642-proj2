#!/bin/bash

# Function to kill all processes and check violations
kill_processes() {
    echo "Shutting down and processing violations..."
    
    # Stop the turtle processes first
    for p in "$@"; do
        if ps -p $p > /dev/null 2>&1; then
            kill $p 2>/dev/null || true
            sleep 1
        fi
    done
    
    # Give time for final messages
    sleep 3
    
    # Ensure VIOLATIONS.txt exists and is readable
    cd "$turtledir/monitors"
    touch VIOLATIONS.txt

    # Terminate monitors gracefully
    pkill -TERM -f "ece642rtle_.*_monitor" 2>/dev/null || true
    sleep 2
    
    # Force kill any remaining monitors
    pkill -9 -f "ece642rtle_.*_monitor" 2>/dev/null || true
    
    echo "=================== FINAL VIOLATIONS SUMMARY ==================="
    
    # Collect warnings from all output files if VIOLATIONS.txt is empty
    if [ ! -s VIOLATIONS.txt ]; then
        for output in *.output.tmp; do
            if [ -f "$output" ]; then
                grep "[ WARN]" "$output" >> VIOLATIONS.txt
            fi
        done
    fi
    
    if [ -s VIOLATIONS.txt ]; then
        cat VIOLATIONS.txt
        echo "===================================================="
        VIOL_COUNT=$(grep -c "\[ WARN\]" VIOLATIONS.txt)
        echo "TOTAL VIOLATIONS: $VIOL_COUNT"
        
        # Count by monitor type
        echo -e "\nViolations by monitor:"
        for monitor in turn tick forward wall face solved atend step; do
            count=$(grep -ci "\[${monitor}\].*VIOLATION" VIOLATIONS.txt)
            if [ $count -gt 0 ]; then
                echo "${monitor^} Monitor: $count violations"
            fi
        done
    else
        echo "No violations found"
    fi
    
    # Clean up temporary files
    rm -f *.output.tmp

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

trap 'kill_processes $ROSCORE_PID' SIGINT
sleep 5

# Set maze file parameter
rosparam set /maze_file "$maze_file"

# Start monitors
echo "Starting monitors..."
cd "$turtledir/monitors"
# Clear any existing monitor outputs
rm -f *.output.tmp VIOLATIONS.txt

# Start monitors with properly captured PID
./run_642_monitors.sh \
    ece642rtle_step_monitor \
    ece642rtle_turn_monitor \
    ece642rtle_tick_monitor \
    ece642rtle_forward_monitor \
    ece642rtle_wall_monitor \
    ece642rtle_face_monitor \
    ece642rtle_solved_monitor \
    ece642rtle_atend_monitor &
MONITOR_PID=$!
sleep 10  # Give monitors more time to start up properly

# Add monitor to trap
trap 'kill_processes $MONITOR_PID $ROSCORE_PID' SIGINT

# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1

if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node"
    kill_processes $MONITOR_PID $ROSCORE_PID
    exit 1
fi

trap 'kill_processes $MONITOR_PID $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!

trap 'kill_processes $MONITOR_PID $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

echo "All processes started. All monitors active. Press Ctrl+C to stop and view violations."

# Wait for Ctrl+C
while [ 1 -eq 1 ]; do
    sleep 30
done
