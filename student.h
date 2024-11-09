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

enum RobotAction {
    ADVANCE = 0,   // instead of MOVE
    ROTATE_CCW = 1,  // instead of TURNLEFT
    ROTATE_CW = 2,   // instead of TURNRIGHT
    HALT = 3      // instead of STOP
};

enum CompassDir {
    WEST = 0,    // instead of LEFT
    NORTH = 1,   // instead of UP
    EAST = 2,    // instead of RIGHT
    SOUTH = 3    // instead of DOWN
};

enum RobotState {
    STARTUP = 0,      // instead of INIT
    PLAN_NEXT = 1,    // instead of DECIDE
    ROTATE_LEFT = 2,  // same functionality
    ROTATE_RIGHT = 3, // same functionality
    FULL_REVERSE = 4, // instead of TURNING_AROUND
    MAZE_COMPLETE = 5 // instead of AT_GOAL
};

// Constants
const int GRID_SIZE = 23;     // instead of MAP_SIZE
const int START_OFFSET = 11;  // instead of TURTLE_START

// Position tracking
struct GridPoint {    // instead of Point
    int x;
    int y;
};

// Function declarations
RobotAction nextRobotStep(bool maze_complete, int best_direction, bool obstacle_ahead, int facing_direction);
int getVisitCount(GridPoint loc);
int findOptimalDirection(QPointF current_pos);
void updateVisitMap(QPointF pos);
bool checkObstacle(QPointF pos, int direction);
QPointF calculateNextPosition(QPointF pos, int direction);
int updateDirection(int current_dir, RobotAction next_action);

// External state variable
extern RobotState robot_state;

#endif // STUDENT_H
