#ifndef STUDENT_H
#define STUDENT_H

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

// Function prototypes to interface with ROS. Don't change these lines!
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool updateTurtle(QPointF& position, int& current_orientation);

// Enumerations for turtle movements, orientations, and states
enum class TurtleAction : int8_t {
    ADVANCE,
    ROTATE_RIGHT,
    ROTATE_LEFT,
    HALT
};

enum class CompassDirection : int8_t {
    WEST,
    SOUTH,
    EAST,
    NORTH
};

enum class TurtleState : int8_t {
    INITIAL,
    MOVING,
    ADJUSTING,
    GOAL_REACHED
};

// Helper functions for position and orientation translations
QPointF computeNextPosition(QPointF current_pos, CompassDirection direction);
int updateOrientation(int orientation, TurtleAction action);

// Turtle decision-making function
TurtleAction determineNextAction(bool is_bumped, bool at_goal, TurtleState* current_state);

// Obstacle detection and visit tracking
bool detectObstacle(QPointF current_pos, CompassDirection direction);
void addVisit(QPointF& position);
uint8_t retrieveVisitCount(QPointF& position);

// Constants
const uint8_t ORIENTATION_COUNT = 4;
const uint8_t MOVE_DELAY = 10;
const uint8_t MAZE_GRID_SIZE = 23;

#endif // STUDENT_H
