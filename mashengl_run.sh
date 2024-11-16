#!/bin/bash

# Function to kill all processes and check violations
kill_processes() {
    echo "Shutting down and checking for violations..."
    
    # Get the actual monitor process name (first 15 chars)
    MONITOR_PROC="ece642rtle_turn_"
    
    # Kill the monitor first and wait for VIOLATIONS.txt
    if [[ -n $(pgrep ${MONITOR_PROC}) ]]; then
        echo "Stopping monitor..."
        pkill -f ${MONITOR_PROC}
        # Wait a moment for VIOLATIONS.txt to be generated
        sleep 3
    fi

    # Kill other processes
    for p in "$@"; do
        if [[ -n $(ps -p $p) ]]; then
            echo "Killing process $p"
            kill $p
            sleep 1
        fi
    done
    
    # Check for violations file
    if [ -f "$turtledir/monitors/VIOLATIONS.txt" ]; then
        echo "=================== VIOLATIONS FOUND ==================="
        cat "$turtledir/monitors/VIOLATIONS.txt"
        echo "===================================================="
        VIOL_COUNT=$(grep -c "VIOLATION:" "$turtledir/monitors/VIOLATIONS.txt")
        echo "TOTAL VIOLATIONS: $VIOL_COUNT"
    else
        echo "TOTAL VIOLATIONS: 0"
    fi
    echo "Any violations logged in VIOLATIONS.txt"
    
    echo "All processes terminated"
    exit 0
}

# ... [rest of your existing setup code] ...

# Start turn monitor correctly
echo "Starting turn monitor..."
cd "$turtledir/monitors"
stdbuf -oL ./run_642_monitors.sh ece642rtle_turn_monitor &
sleep 2

# ... [rest of your existing process startup code] ...

# Make sure we're using the full process management in the trap
trap 'kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

echo "All processes started. Monitor active. Press Ctrl+C to stop and view violations."

# Wait for Ctrl+C
while [ 1 -eq 1 ]; do
    sleep 30
done
