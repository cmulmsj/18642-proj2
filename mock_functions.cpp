/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <iostream>
#include <cstdarg>

// State variables
RobotState robot_state = STARTUP;
bool first_run = true;
coordinate current_pos = {START_POS, START_POS};
int facing_direction = 1;
int rotations_checked = 0;
int best_direction = -1;

// Mock state variables
static bool mock_wall = false;
static int mock_visit_count = 0;

// Test control functions
void mock_set_wall(bool has_wall) {
    mock_wall = has_wall;
}

void mock_set_visit_count(int count) {
    mock_visit_count = count;
}

void mock_reset_state() {
    mock_wall = false;
    mock_visit_count = 0;
    robot_state = STARTUP;
    first_run = true;
    current_pos = {START_POS, START_POS};
    facing_direction = 1;
    rotations_checked = 0;
    best_direction = -1;
}

// Mock implementations
bool checkObstacle(QPointF pos, int direction) {
    return mock_wall;
}

int getVisitCount(coordinate pos) {
    return mock_visit_count;
}

void ROS_INFO(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
