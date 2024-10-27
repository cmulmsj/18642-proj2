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

// Simple coordinate struct
typedef struct {
    uint8_t x;
    uint8_t y;
} coordinate;

// Action enum - Kept simple for clarity
enum turtleAction {
    FORWARD,    // Move forward one cell
    RIGHT,      // Turn right 90 degrees
    LEFT,       // Turn left 90 degrees
    NONE        // No action/stop
};

// Movement message struct
typedef struct {
    turtleAction action;
    bool validAction;
    uint8_t visitCount;
} turtleMove;

// Core function declarations - Required interface
QPointF translatePos(QPointF pos_, turtleMove nextMove, int compass_orientation);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped, bool at_end);

// Helper function declarations
uint8_t getVisits(coordinate pos);
void setVisits(coordinate pos, uint8_t val);
void updatePosition(coordinate& pos, uint8_t direction);
bool checkWall(coordinate pos, uint8_t direction);

// Constants for maze navigation
constexpr uint8_t MAZE_SIZE = 30;
constexpr uint8_t START_POS = 14;
constexpr uint8_t MOVE_WAIT = 5;
constexpr uint8_t MAX_DIRECTIONS = 4;

#endif // STUDENT_H
