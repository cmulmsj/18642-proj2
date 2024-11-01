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

# Initialize catkin workspace
echo "Initializing catkin workspace..."
cd ~/catkin_ws
catkin_make

# Source the setup file
echo "Sourcing setup file..."
source devel/setup.bash

# Build the project
echo "Building project..."
catkin_make ece642rtle_student

# Clean up extracted files
echo "Cleaning up..."
cd ~
rm -rf project_files

echo "Build completed successfully!"
