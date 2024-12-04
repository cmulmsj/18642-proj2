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

# Directory where this script is located
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# Store monitor names in array
MONITORS=("$@")

# Cleanup function for existing processes
cleanup() {
    pkill -f "ece642rtle_.*_monitor" || true
    rm -f *.output.tmp
    rm -f VIOLATIONS.txt
}

# Run cleanup on start
cleanup
sleep 1

# Create named pipes for each monitor
for monitor in "${MONITORS[@]}"; do
    # Create named pipe if it doesn't exist
    PIPE="/tmp/${monitor}_pipe"
    [[ -p $PIPE ]] || mkfifo $PIPE
done

# Start monitors and set up output handling
for monitor in "${MONITORS[@]}"; do
    PIPE="/tmp/${monitor}_pipe"
    
    # Start monitor and tee output to both pipe and temp file
    rosrun ece642rtle "$monitor" 2>&1 | tee "$monitor.output.tmp" > "$PIPE" &
    
    # Read from pipe and display in real time, prefixed with monitor name
    # Also save warnings to VIOLATIONS.txt
    (while read -r line; do
        # Display all output
        echo "[$monitor] $line"
        
        # Save warnings to VIOLATIONS.txt
        if [[ $line == *"WARN"* ]]; then
            echo "[$monitor] $line" >> VIOLATIONS.txt
        fi
    done < "$PIPE") &
done

# Cleanup function for SIGINT/SIGTERM
trap cleanup EXIT

# Wait for monitors
wait

# Clean up pipes
for monitor in "${MONITORS[@]}"; do
    rm -f "/tmp/${monitor}_pipe"
done
