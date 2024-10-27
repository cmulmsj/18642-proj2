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
bool bumped(int start_x, int start_y, int end_x, int end_y);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Movement Commands
enum turtleMove : int8_t {
    MOVING,
    TURNING_RIGHT,
    TURNING_LEFT,
    STOPPING
};

// Cardinal Directions
enum Orientation : int8_t {
    WEST,
    SOUTH,
    EAST,
    NORTH
};

// Turtle States for Priority-Based Navigation
enum TurtleState : int8_t {
    INITIALIZING,    // Initial state
    SCANNING,        // Evaluating surroundings
    MOVING_STATE,    // Moving forward
    TURNING_STATE,   // Executing a turn
    BACKTRACKING,    // Reversing from dead end
    GOAL_REACHED     // Reached target
};

// Direction finding helper
struct AdjacentSquare {
    uint8_t visit_count;   // Number of visits to this square
    bool is_accessible;    // Whether this square can be reached
    Orientation direction; // Direction to this square
};

// Function Declarations
QPointF translatePos(QPointF pos_, Orientation orientation);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state);

// Position and navigation helpers
bool detectObstacle(QPointF pos_, Orientation orient);
void addVisit(QPointF& pos_);
uint8_t retrieveVisitCount(QPointF& pos_);
AdjacentSquare findBestMove(QPointF pos_, Orientation current_orient);

// Constants
const uint8_t ORIENTATION_COUNT = 4;   // Number of possible orientations
const uint8_t MOVE_DELAY = 40;         // Delay between moves
const uint8_t MAZE_GRID_SIZE = 23;     // Size of the maze grid
const uint8_t CENTER_POS = 11;         // Center position (MAZE_GRID_SIZE/2)

#endif // STUDENT_H
