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
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Enumerations for turtle moves and orientations
enum TurtleMove : int8_t {
    ADVANCE,
    ROTATE_CW,
    ROTATE_CCW,
    HALT
};

enum TurtleDirection : int8_t {
    WEST,  // 0
    SOUTH, // 1
    EAST,  // 2
    NORTH  // 3
};

enum NavigationMode : int8_t {
    INITIAL,
    PROCEED,
    ADJUST,
    COMPLETE
};

// Function prototypes with updated naming conventions
QPointF translatePos(QPointF pos_, TurtleDirection orientation);
int translateOrnt(int orientation, TurtleMove nextMove);
TurtleMove studentTurtleStep(bool bumped, bool goal, NavigationMode* cur_state);

bool detectObstacle(QPointF pos_, TurtleDirection orient);
void addVisit(QPointF& pos_);
uint8_t getVisit(QPointF& pos_);

const uint8_t NUM_ORIENTATIONS = 4;
const uint8_t TIMEOUT = 40;
const uint8_t MAZE_SIZE = 23;

#endif // STUDENT_H
