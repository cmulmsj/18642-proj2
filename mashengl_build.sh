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

# Make test scripts executable
echo "Setting execute permissions..."
chmod +x ~/catkin_ws/src/ece642rtle/turtle_tests/mashengl_build_run_tests.sh

# Initialize catkin workspace and build main project
echo "Building main project..."
cd ~/catkin_ws
catkin_make ece642rtle_student -Wall -Werror

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

# Clean up extracted files
echo "Cleaning up..."
cd ~
rm -rf project_files

echo "Build completed!"
echo "Note: Unit test results are shown above between the delimiter lines."
