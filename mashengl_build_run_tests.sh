#!/bin/bash

# Build and run unit tests for turtle implementation
# Student: Mashengjun Li (mashengl)

g++ -Dtesting -o student_tests \
    mashengl_student_test.cpp \
    mashengl_student_turtle.cpp \
    mock_functions.cpp \
    -lcunit

if [ $? -eq 0 ]; then
    echo "Build successful, running tests..."
    ./student_tests
else
    echo "Build failed!"
    exit 1
fi
