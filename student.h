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

// Constants
const int GRID_SIZE = 30;
const int START_POS = 13;  // Starting position (middle of grid)
static const int TIMEOUT = 40;  // Timeout cycles for controlling speed

// External functions
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);

// Enums and Structs
enum turtleAction {
    FORWARD,
    LEFT,
    RIGHT
};

typedef struct {
    uint8_t x;
    uint8_t y;
} coordinate;

typedef struct {
    turtleAction action;
    bool validAction;
    uint8_t visitCount;
} turtleMove;

// Core functions
turtleMove studentTurtleStep(bool bumped, bool at_end);
bool moveTurtle(QPointF& pos, int& orientation);
bool checkObstacle(QPointF pos, int direction);

#endif // STUDENT_H
