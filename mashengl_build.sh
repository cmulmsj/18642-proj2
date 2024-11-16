#!/bin/bash

# Exit on any error
set -e

echo "Starting build process..."

# Extract the inner tarball containing project files
echo "Extracting project files..."
tar -zxvf mashengl_files.tgz

# Create catkin workspace if it doesn't exist
if [ ! -d "~/catkin_ws" ]; then
    echo "Creating catkin workspace..."
    mkdir -p ~/catkin_ws/src
fi

# Copy project files to appropriate locations
echo "Setting up project files..."
cp -r project_files/ece642rtle ~/catkin_ws/src/

# Make scripts executable
echo "Setting execute permissions..."
chmod +x ~/catkin_ws/src/ece642rtle/turtle_tests/mashengl_build_run_tests.sh
chmod +x ~/catkin_ws/src/ece642rtle/monitors/run_642_monitors.sh

# Initialize catkin workspace and build main project
echo "Building main project..."
cd ~/catkin_ws
catkin_make ece642rtle_student -Wall -Werror

# Build monitor
echo "Building turn monitor..."
catkin_make ece642rtle_turn_monitor

# Run unit tests
echo "========================================"
echo "Running unit tests..."
cd src/ece642rtle/turtle_tests
./mashengl_build_run_tests.sh
TEST_RESULT=$?

if [ $TEST_RESULT -ne 0 ]; then
    echo "Warning: Some unit tests failed! Check the output above for details."
    echo "The build will continue, but please review test failures."
else
    echo "All unit tests passed successfully!"
fi
echo "========================================"

# Source the setup file
echo "Sourcing setup file..."
cd ~/catkin_ws
source devel/setup.bash

# Run monitor and print status
echo "========================================"
echo "Monitor status:"
cd src/ece642rtle/monitors
if [ -f "VIOLATIONS.txt" ]; then
    echo "Previous VIOLATIONS.txt exists, removing..."
    rm VIOLATIONS.txt
fi
echo "Monitor can be run using:"
echo "cd ~/catkin_ws/src/ece642rtle/monitors"
echo "./run_642_monitors.sh ece642rtle_turn_monitor"
echo "========================================"

echo "Build completed!"
echo "Note: Unit test results and monitor status are shown above between the delimiter lines."
echo "Project files and build artifacts preserved for monitoring."
