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
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted
enum turtleMove { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, STOP };
turtleMove studentTurtleStep(bool bumped);
void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove);
void translateOrnt(int& nw_or, turtleMove nextMove);
int getCurrentVisitCount();

// OK to change below this line
// No further changes needed

#endif // STUDENT_H