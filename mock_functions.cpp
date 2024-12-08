/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// State variables
RobotState robot_state = STARTUP;
bool first_run = true;
coordinate current_pos = {START_POS, START_POS};
int facing_direction = 1;
int rotations_checked = 0;
int best_direction = -1;
int min_visits = UINT8_MAX;
int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};

// Mock state variables
static bool mock_wall = false;
static int mock_visit_count = 0;

// Mock functions
void mock_reset_state() {
    robot_state = STARTUP;
    first_run = true;
    current_pos = {START_POS, START_POS};
    facing_direction = 1;
    rotations_checked = 0;
    best_direction = -1;
    min_visits = UINT8_MAX;
    memset(visit_grid, 0, sizeof(visit_grid));
    mock_wall = false;
    mock_visit_count = 0;
}

void mock_set_wall(bool has_wall) {
    mock_wall = has_wall;
}

bool mock_get_wall() {
    return mock_wall;
}

void mock_set_visit_count(int count) {
    mock_visit_count = count;
}

void mock_ros_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    printf("\n");
    va_end(args);
}

void mock_ros_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    fprintf(stderr, "ERROR: ");
    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
    va_end(args);
}

bool checkObstacle(QPointF pos, int direction) {
    return mock_wall;
}

int getVisitCount(coordinate pos) {
    return mock_visit_count;
}

void updateVisitMap(coordinate pos) {
    if (pos.x >= 0 && pos.x < GRID_SIZE && pos.y >= 0 && pos.y < GRID_SIZE) {
        visit_grid[pos.x][pos.y]++;
        mock_visit_count = visit_grid[pos.x][pos.y];
    }
}
