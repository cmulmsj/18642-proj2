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

// Enum to define turtle's possible moves
enum turtleMove { FORWARD, LEFT, RIGHT };

// Function declarations
bool moveTurtle(QPointF& pos_, int& nw_or);
turtleMove studentTurtleStep(bool bumped);
QPointF translatePos(QPointF pos_, turtleMove nextMove);
int translateOrnt(int orientation, turtleMove nextMove);

// Wall-checking and goal functions (simulated by the maze logic)
bool bumped(int x1, int y1, int x2, int y2);
bool atEnd(int x, int y);
void displayVisits(int visits);

// Getter and setter for the visit count in the local map
int getVisitCount(int x, int y);
void setVisitCount(int x, int y, int visits);

#endif // STUDENT_H
