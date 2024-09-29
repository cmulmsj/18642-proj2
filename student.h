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

enum turtleMove : int8_t {
    MOVING,
    TURNING_RIGHT,
    TURNING_LEFT,
    STOPPING
};

enum Orientation : int8_t {
    WEST,
    SOUTH,
    EAST,
    NORTH
};

enum TurtleState : int8_t {
    INIT,
    GO,
    TURN,
    GOAL
};

QPointF translatePos(QPointF pos_, Orientation orientation);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state);

bool detectObstacle(QPointF pos_, Orientation orient);
void addVisit(QPointF& pos_);
uint8_t retrieveVisitCount(QPointF& pos_);

const uint8_t ORIENTATION_COUNT = 4;
const uint8_t MOVE_DELAY = 10;
const uint8_t MAZE_GRID_SIZE = 23;

#endif // STUDENT_H
