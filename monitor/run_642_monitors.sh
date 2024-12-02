#!/bin/bash

# Kill any existing monitor processes
pkill -f "ece642rtle_.*_monitor"
sleep 1

# Array of monitor names
MONITORS=(
    "ece642rtle_step_monitor"
    "ece642rtle_turn_monitor"
    "ece642rtle_tick_monitor"
    "ece642rtle_forward_monitor"
    "ece642rtle_wall_monitor"
    "ece642rtle_face_monitor"
    "ece642rtle_solved_monitor"
    "ece642rtle_atend_monitor"
)

# Start all monitors and record their output
for monitor in "${MONITORS[@]}"; do
    rosrun ece642rtle "$monitor" > "${monitor}_output.tmp" 2>&1 &
done

# Wait a moment for monitors to start
sleep 2

# When script is interrupted
cleanup() {
    echo "Cleaning up..."
    pkill -f "ece642rtle_.*_monitor"
    
    # Process violations
    > VIOLATIONS.txt  # Create/clear violations file
    
    # Process each monitor's output
    for monitor in "${MONITORS[@]}"; do
        if [ -f "${monitor}_output.tmp" ]; then
            echo "Processing ${monitor}..."
            # Get 5 lines before and after each violation
            grep -B 5 -A 5 "VIOLATION:" "${monitor}_output.tmp" >> VIOLATIONS.txt
            echo "----------------------------------------" >> VIOLATIONS.txt
        fi
    done
    
    # Count total violations
    TOTAL=$(grep -c "VIOLATION:" VIOLATIONS.txt)
    echo "Total violations found: $TOTAL"
    
    # Cleanup temp files
    rm -f *_output.tmp
    
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep script running
while true; do
    sleep 1
done
