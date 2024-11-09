#!/bin/bash
# Build and run tests for turtle implementation
# Student Name: Mashengjun Li
# Andrew ID: mashengl

# Compile with testing flag and C++11 support
g++ -std=c++11 -Dtesting \
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
