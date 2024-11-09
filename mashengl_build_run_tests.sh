#!/bin/bash

# Build and run unit tests for turtle implementation
# Student: Mashengjun Li (mashengl)

g++ -std=c++11 -Dtesting \
    -I/opt/ros/kinetic/include \
    -o mashengl_student_tests \
    mashengl_student_test.cpp \
    mashengl_student_turtle.cpp \
    mock_functions.cpp \
    -lcunit

# Check if compilation succeeded
if [ $? -eq 0 ]; then
    echo "Build successful, running tests..."
    ./mashengl_student_tests
else
    echo "Build failed!"
    exit 1
fi
