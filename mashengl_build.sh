#!/bin/bash

# Exit on any error
set -e

echo "Starting build process..."

# Project specific paths
PROJECT_DIR="/home/student/18642-proj2"
cd $PROJECT_DIR

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

# Initialize catkin workspace and build
cd ~/catkin_ws

# Build student node
echo "Building student node..."
catkin_make ece642rtle_student -Wall -Werror

# Build monitor target
echo "Building monitor..."
catkin_make ece642rtle_turn_monitor

# Run unit tests
echo "========================================"
echo "Running unit tests..."
cd src/ece642rtle/turtle_tests
./mashengl_build_run_tests.sh
TEST_RESULT=$?

if [ $TEST_RESULT -ne 0 ]; then
    echo "Warning: Some unit tests failed! Check the output above for details."
else
    echo "All unit tests passed successfully!"
fi
echo "========================================"

# Source the setup file
echo "Sourcing setup file..."
cd ~/catkin_ws
source devel/setup.bash

# Return to project directory
cd $PROJECT_DIR

echo "Build completed!"
