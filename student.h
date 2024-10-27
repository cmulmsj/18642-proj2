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

// Functions to interface with ROS. Don't change these lines!
bool bumped(int start_x, int start_y, int end_x, int end_y);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Basic point structure for grid coordinates
struct Point {
    int32_t x;
    int32_t y;
};

// Movement Commands - DO NOT MODIFY THESE VALUES
enum turtleMove : int8_t {
    MOVE = 0,           // Move forward one square
    TURNRIGHT = 1,      // Turn 90 degrees clockwise
    TURNLEFT = 2,       // Turn 90 degrees counter-clockwise
    STOP = 3           // Stop moving
};

// Cardinal Directions
enum Orientation : int8_t {
    WEST = 0,
    SOUTH = 1,
    EAST = 2,
    NORTH = 3
};

// Turtle's FSM States
enum TurtleState : int8_t {
    MOVE_FORWARD = 0,    // Moving forward
    TURN_RIGHT = 1,      // Turning right
    TURN_LEFT = 2,       // Turning left
    CHECK_SURROUNDINGS = 3, // Checking surroundings
    AT_END = 4           // Reached the goal
};

// Function Declarations
QPointF translatePos(QPointF pos_, Orientation orientation);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state);

// Position and navigation helpers
bool detectObstacle(QPointF pos_, Orientation orient);
void addVisit(QPointF& pos_);
uint8_t retrieveVisitCount(QPointF& pos_);
int getVisitNumber(Point &pos_);

// Constants
const uint8_t ORIENTATION_COUNT = 4;   // Number of possible orientations
const uint8_t MOVE_DELAY = 40;         // Delay between moves
const uint8_t MAZE_GRID_SIZE = 23;     // Size of the maze grid
const uint8_t CENTER_POS = 11;         // Center position (MAZE_GRID_SIZE/2)

#endif // STUDENT_H
