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
    echo "Starting $monitor..."
    rosrun ece642rtle "$monitor" > "${monitor}_output.tmp" 2>&1 &
done

# Wait a moment for monitors to start
sleep 2

# When script is interrupted
cleanup() {
    echo "Cleaning up..."
    pkill -f "ece642rtle_.*_monitor"
    
    echo "Creating VIOLATIONS.txt..."
    rm -f VIOLATIONS.txt  # Remove if exists
    touch VIOLATIONS.txt  # Create new file
    
    # Process each monitor's output
    for monitor in "${MONITORS[@]}"; do
        echo "Checking ${monitor}_output.tmp..."
        if [ -f "${monitor}_output.tmp" ]; then
            echo "Found output file for $monitor"
            echo "=== Violations from $monitor ===" >> VIOLATIONS.txt
            # First, print the raw content to see what we're working with
            echo "Raw content of ${monitor}_output.tmp:" >> VIOLATIONS.txt
            cat "${monitor}_output.tmp" >> VIOLATIONS.txt
            echo "----------------------------------------" >> VIOLATIONS.txt
            
            # Now try to extract violations
            echo "Extracting violations..." >> VIOLATIONS.txt
            grep -B 5 -A 5 "VIOLATION:" "${monitor}_output.tmp" >> VIOLATIONS.txt 2>/dev/null || echo "No violations found in $monitor"
            echo "----------------------------------------" >> VIOLATIONS.txt
        else
            echo "Warning: ${monitor}_output.tmp not found!"
        fi
    done
    
    # Count total violations
    if [ -f VIOLATIONS.txt ]; then
        TOTAL=$(grep -c "VIOLATION:" VIOLATIONS.txt)
        echo "Total violations found: $TOTAL"
        echo "Check VIOLATIONS.txt for details"
    else
        echo "Error: VIOLATIONS.txt was not created!"
    fi
    
    # Don't clean up temp files yet - leave them for inspection
    echo "Temporary output files preserved for debugging"
    
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Monitors started. Press Ctrl+C to stop and generate violation report."

# Keep script running
while true; do
    sleep 1
done
