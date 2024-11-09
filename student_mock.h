/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <cstdint>

enum turtleAction {FORWARD, LEFT, RIGHT};
enum RobotState {STARTUP, PLAN_NEXT, MOVING};

typedef struct {
    uint8_t x;
    uint8_t y;
} coordinate;

typedef struct {
    turtleAction action;
    bool validAction;
    uint8_t visitCount;
} turtleMove;

// Core function to test
turtleMove studentTurtleStep(bool bumped, bool at_end);

// Mock functions for testing
void mock_set_wall(bool has_wall);
void mock_set_visit_count(int count);
void mock_set_directional_visits(int north, int east, int south, int west);
int getVisitCount(coordinate pos);
int getExpectedX();
int getExpectedY();

// External state variables that need to be accessible
extern RobotState robot_state;
extern bool first_run;
extern coordinate current_pos;
extern int facing_direction;
extern int rotations_checked;
extern int best_direction;

#endif // STUDENT_MOCK_H
