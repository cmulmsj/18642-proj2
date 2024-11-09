/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <iostream>
#include <cstdarg>

static int mock_visit_count = 0;
static bool mock_wall = false;

RobotState robot_state = STARTUP;
bool first_run = true;
coordinate current_pos = {START_POS, START_POS};
int facing_direction = 1;
int rotations_checked = 0;
int best_direction = -1;

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

int getExpectedX() {
    return current_pos.x;
}

int getExpectedY() {
    return current_pos.y;
}

// Mock ROS_INFO implementation
void ROS_INFO(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
