/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <iostream>
#include <cstdarg>

// Mock state variables
static bool mock_wall = false;
static int mock_visit_count = 0;

// Mock functions for test control
void mock_set_wall(bool has_wall) {
    mock_wall = has_wall;
}

void mock_set_visit_count(int count) {
    mock_visit_count = count;
}

void mock_reset_state() {
    mock_wall = false;
    mock_visit_count = 0;
}

// Mock implementation of original functions
bool checkObstacle(QPointF pos, int direction) {
    return mock_wall;
}

void ROS_INFO(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
