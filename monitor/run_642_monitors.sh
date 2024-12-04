# #!/bin/bash

# # Directory where this script is located
# DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# cd "$DIR"

# # Initialize/clear output files
# for monitor in "$@"; do
#     > "${monitor}.output.tmp"
# done

# # Store monitor names in an array
# MONITORS=("$@")

# # Start each monitor
# for monitor in "${MONITORS[@]}"; do
#     echo "Starting $monitor..."
#     rosrun ece642rtle "$monitor" > "${monitor}.output.tmp" 2>&1 &
#     sleep 2  # Increased sleep between monitor starts
# done

# # Process for collecting violations
# collect_violations() {
#     rm -f VIOLATIONS.txt
    
#     # Process outputs
#     for monitor in "${MONITORS[@]}"; do
#         if [ -f "${monitor}.output.tmp" ]; then
#             echo "=== Violations from $monitor ===" >> VIOLATIONS.txt
#             grep -C 5 "\[ WARN\]" "${monitor}.output.tmp" >> VIOLATIONS.txt
#         fi
#     done
# }

# # Set up signal handler - properly formatted
# trap collect_violations SIGINT SIGTERM

# # Keep running until parent terminates
# wait

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
