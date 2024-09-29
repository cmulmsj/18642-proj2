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
#include "mashengl_maze_params.h"

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

QPointF updatePosition(QPointF pos_, int orientation);
int updateOrientation(int orientation, TurtleCommand nextCommand);
TurtleCommand decideTurtleAction(bool obstacle_detected, bool goal_reached, NavigationMode* current_mode);

bool detectObstacle(QPointF pos_, int facing);
void logVisit(QPointF& pos_);
uint8_t getVisitCount(QPointF& pos_);

#endif // STUDENT_H
