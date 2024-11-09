/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <iostream>

static int mock_visit_count = 0;
static bool mock_wall = false;

// Mock function for visit count
int getVisitCount(coordinate pos) {
    return mock_visit_count;
}

// Mock function for wall detection
bool checkObstacle(QPointF pos, int direction) {
    return mock_wall;
}

// Test control functions
void mock_set_wall(bool has_wall) {
    mock_wall = has_wall;
}

void mock_set_visit_count(int count) {
    mock_visit_count = count;
}

// Dummy ROS_INFO implementation for testing
void ROS_INFO(const char* format, ...) {
    // Do nothing in tests
}
