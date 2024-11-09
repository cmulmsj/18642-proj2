/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <cstdint>
#include <climits>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Constants from original student.h
#define GRID_SIZE 30
#define START_POS 13

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

// Core function to test
turtleMove studentTurtleStep(bool bumped, bool at_end);

// Mock functions for testing
void mock_set_wall(bool has_wall);
void mock_set_visit_count(int count);
void mock_set_directional_visits(int north, int east, int south, int west);
int getVisitCount(coordinate pos);
int getExpectedX();
int getExpectedY();
bool checkObstacle(QPointF pos, int direction);

// ROS mock
void ROS_INFO(const char* format, ...);

// External state variables that need to be accessible
extern RobotState robot_state;
extern bool first_run;
extern coordinate current_pos;
extern int facing_direction;
extern int rotations_checked;
extern int best_direction;

#endif // STUDENT_MOCK_H
