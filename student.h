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
    MOVE,
    TURNRIGHT,
    TURNLEFT,
    STOP
};

enum Orientation : int8_t {
    LEFT,
    DOWN,
    RIGHT,
    UP
};

enum State : int8_t {
    INIT,
    GO,
    TURN,
    GOAL
};

QPointF translatePos(QPointF pos_, Orientation orientation);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped, bool goal, State* cur_state);

bool check_bumped(QPointF pos_, Orientation orient);
void record_visited(QPointF& pos_);
uint8_t get_visited(QPointF& pos_);

const uint8_t NUM_ORIENTATIONS = 4;
const uint8_t TIMEOUT = 40;
const uint8_t MAZE_SIZE = 23;

#endif // STUDENT_H
