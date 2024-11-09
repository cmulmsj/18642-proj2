#!/bin/bash

echo "Building and running turtle navigation unit tests..."

g++ -Dtesting -o student_tests \
    mashengl_student_test.cpp \
    student_turtle.cpp \
    mock_functions.cpp \
    -lcunit

if [ $? -eq 0 ]; then
    echo "Build successful. Running tests..."
    ./student_tests
else
    echo "Build failed!"
    exit 1
fi
