#!/bin/bash

# Kill any existing monitor processes
pkill -f "ece642rtle_.*_monitor"
sleep 2  # Increased sleep time

# Start each monitor and redirect output
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

for m in "${MONITORS[@]}"; do
    rosrun ece642rtle "$m" > "$m.output.tmp" 2>&1 &
    sleep 1  # Give each monitor time to start properly
done

# Cleanup function
cleanup() {
    echo "Processing monitor outputs..."
    rm -f VIOLATIONS.txt
    
    for m in "${MONITORS[@]}"; do
        if [ -f "$m.output.tmp" ]; then
            echo "Processing $m..."
            grep -C 5 "\[ WARN\]" "$m.output.tmp" >> VIOLATIONS.txt
            m_viol=$(grep "\[ WARN\]" "$m.output.tmp" | wc -l)
            echo "$m: $m_viol violations"
        fi
    done
    
    # Keep output files for inspection
    echo "Monitor outputs preserved in *.output.tmp files"
}

trap cleanup SIGINT SIGTERM

# Keep script running
while true; do
    sleep 1
done
