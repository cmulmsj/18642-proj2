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
bool moveTurtle(QPointF& pos_, int32_t &orientation);

// Scope-preserving changes to these lines permitted (see p5 writeup)
enum turtleMove {
    TURNRIGHT = 0,
    TURNLEFT = 1,
    MOVE = 2,
    STOP = 3
};

// Translate the turtle's position and orientation
QPointF translatePos(QPointF pos_, int32_t orientation);

// Translate the turtle's orientation based on the next move
int translateOrnt(int32_t orientation, turtleMove nextMove);

// Updated function for the turtle's movement logic based on the greedy algorithm
turtleMove studentTurtleStep(bool at_end, int32_t least_visited_direction, bool bump, int32_t orientation);

// OK to change below this line
typedef struct {
    int32_t x;
    int32_t y;
} Point;

enum Orientation {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

enum MovementState {
    FORWARD = 0,
    TURN_RIGHT = 1,
    TURN_AROUND = 2,
    TURN_LEFT = 3
};

enum TurtleState {
    INIT = 0,
    DECIDE = 1,
    TURNING_LEFT = 2,
    TURNING_RIGHT = 3,
    TURNING_AROUND = 4,
    AT_GOAL = 5
};

// Get visit count for a specific position in the maze
int getVisitNumber(Point &pos_);

// Function to find the least-visited valid direction for the turtle to move
int getLeastVisitedDirection(QPointF pos, int32_t orientation);

// Record the turtle's path in the maze
void record_path(QPointF &pos_);

// Detect if bump in the orientation of the turtle
bool detectBump(QPointF &pos_, int32_t orientation);

extern TurtleState state;
