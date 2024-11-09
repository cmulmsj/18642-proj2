/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <cstdint>
#include <climits>
#include <iostream>

// Constants
#define GRID_SIZE 30
#define START_POS 13

// Mock ROS logging macros
#define ROS_INFO(...) mock_ros_info(__VA_ARGS__)
#define ROS_ERROR(...) mock_ros_error(__VA_ARGS__)

// Mock QPointF
class QPointF {
public:
    QPointF() : xpos(0), ypos(0) {}
    QPointF(double x, double y) : xpos(x), ypos(y) {}
    double x() const { return xpos; }
    double y() const { return ypos; }
    void setX(double x) { xpos = x; }
    void setY(double y) { ypos = y; }
private:
    double xpos, ypos;
};

enum turtleAction {FORWARD, LEFT, RIGHT};
enum RobotState {STARTUP, PLAN_NEXT, MOVING};

struct coordinate {
    int x;
    int y;
};

struct turtleMove {
    turtleAction action;
    bool validAction;
    int visitCount;
};

// Main function that we're testing
turtleMove studentTurtleStep(bool bumped, bool at_end);

// Test control functions
void mock_set_wall(bool has_wall);
bool mock_get_wall();  // Add this getter
void mock_set_visit_count(int count);
void mock_reset_state();
void updateVisitMap(coordinate pos);

// Original functions that need mocking
bool checkObstacle(QPointF pos, int direction);
int getVisitCount(coordinate pos);
void ROS_INFO(const char* format, ...);

// Add function declarations for mock logging
void mock_ros_info(const char* format, ...);
void mock_ros_error(const char* format, ...);

// State variables
extern RobotState robot_state;
extern bool first_run;
extern coordinate current_pos;
extern int facing_direction;
extern int rotations_checked;
extern int best_direction;
extern int min_visits;
extern int visit_grid[GRID_SIZE][GRID_SIZE];

#endif // STUDENT_MOCK_H
