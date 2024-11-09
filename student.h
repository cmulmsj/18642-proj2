// #ifndef STUDENT_H
// #define STUDENT_H

// #include <ros/ros.h>
// #include <boost/bind.hpp>
// #include <ece642rtle/timeInt8.h>
// #include <std_msgs/Empty.h>
// #include <ece642rtle/RTIbump.h>
// #include <ece642rtle/RTIatend.h>
// #include <ece642rtle/PoseOrntBundle.h>
// #include <ece642rtle/bumpEcho.h>
// #include <ece642rtle/aendEcho.h>
// #include <QPointF>

// // Functions to interface with ROS. Don't change these lines!
// bool bumped(int x1,int y1,int x2,int y2);
// bool atend(int x, int y);
// void displayVisits(int visits);
// bool moveTurtle(QPointF& pos_, int& nw_or);

// //enums for the possible turtleActions
// enum turtleAction {FORWARD, LEFT, RIGHT};

// //typedef for coordinates (x,y) in the map grid
// typedef struct {
//   uint8_t x;
//   uint8_t y;
// } coordinate;

// //typedef for turtleMove message passed between turtle logic and maze navigation algorithms
// //contains action: to move FORWARD | LEFT | RIGHT
// //validAction: If the action is valid or not
// //visitCount: The number of visits it has made to this next square 
// typedef struct {
//     turtleAction action;
//     bool validAction;
//     uint8_t visitCount;
// } turtleMove;

// void translatePosAndOrientation(turtleMove nextMove, QPointF& pos_, int& compass_orientation);
// QPointF translatePos(QPointF pos_, turtleMove nextMove, int compass_orientation);
// int translateOrnt(int orientation, turtleMove nextMove);
// turtleMove studentTurtleStep(bool bumped, bool at_end);


// #endif // STUDENT_H

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

// External functions provided by framework
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);

// Turtle movement enums and structs
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

// Core navigation functions
turtleMove studentTurtleStep(bool bumped, bool at_end);
bool moveTurtle(QPointF& pos, int& orientation);

#endif // STUDENT_H
