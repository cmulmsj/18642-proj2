#!/bin/bash

# Store monitor names in array
MONITORS=("$@")

# Cleanup any existing monitor processes and files
pkill -f "ece642rtle_.*_monitor" 2>/dev/null || true
rm -f *.output.tmp VIOLATIONS.txt
sleep 2

# Create fresh VIOLATIONS.txt
touch VIOLATIONS.txt

# Start each monitor and redirect output
for monitor in "${MONITORS[@]}"; do
    # Start monitor and tee output to both terminal and file
    rosrun ece642rtle "$monitor" 2>&1 | tee -a "$monitor.output.tmp" | while read -r line; do
        # Echo all output to terminal
        echo "[$monitor] $line"
        
        # Save warnings to VIOLATIONS.txt
        if [[ "$line" == *"[ WARN]"* ]]; then
            echo "[$monitor] $line" >> VIOLATIONS.txt
        fi
    done &
done

# Just wait - let the parent script handle cleanup
wait
