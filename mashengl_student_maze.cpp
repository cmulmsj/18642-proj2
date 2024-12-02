/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/08/2024
 */

#include "student.h"
#include <cmath>
#include <ros/ros.h>

// Global variables for tick synchronization
static bool tick_active = false;
static bool action_done = false;

bool checkObstacle(QPointF pos, int direction) {
    int x = static_cast<int>(std::floor(pos.x()));
    int y = static_cast<int>(std::floor(pos.y()));
    int x1 = x, y1 = y;
    int x2 = x, y2 = y;

    switch (direction) {
        case 0: // WEST
            y2 = y + 1;
            break;
        case 1: // NORTH
            x2 = x + 1;
            break;
        case 2: // EAST
            x1 = x + 1;
            x2 = x + 1;
            y2 = y + 1;
            break;
        case 3: // SOUTH
            x2 = x + 1;
            y1 = y + 1;
            y2 = y + 1;
            break;
        default:
            ROS_ERROR("Invalid direction in checkObstacle");
            return true; // Treat invalid direction as obstacle
    }
    return bumped(x1, y1, x2, y2);
}

// bool moveTurtle(QPointF& pos, int& orientation) {
//     // Add delay for timeout
//     static const double MOVE_DELAY = 0.2;
//     ros::Duration(MOVE_DELAY).sleep();  // 200ms delay
    
//     // Check for wall and goal
//     bool wall_detected = checkObstacle(pos, orientation);
//     bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
//                              static_cast<int>(std::floor(pos.y())));
    
//     // Get next move
//     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
//     if (!next_move.validAction) {
//         return false;
//     }
    
//     // Execute move
//     switch (next_move.action) {
//         case FORWARD:
//             if (!wall_detected) {
//                 switch (orientation) {
//                     case 0: pos.setX(pos.x() - 1.0); break;
//                     case 1: pos.setY(pos.y() - 1.0); break;
//                     case 2: pos.setX(pos.x() + 1.0); break;
//                     case 3: pos.setY(pos.y() + 1.0); break;
//                     default:
//                         ROS_ERROR("Invalid orientation in moveTurtle");
//                         return false;
//                 }
//                 displayVisits(next_move.visitCount);
//             }
//             break;
            
//         case RIGHT:
//             orientation = (orientation + 1) % 4;
//             break;
            
//         case LEFT:
//             orientation = (orientation + 3) % 4;
//             break;
            
//         default:
//             ROS_ERROR("Invalid action in moveTurtle");
//             return false;
//     }
    
//     return true;
// }

void startTick() {
    if (tick_active) {
        ROS_WARN("Tick already active. Possible logic error.");
    }
    tick_active = true;
    action_done = false;  // Reset action flag for the new tick
    ROS_INFO("Tick started.");
}

void endTick() {
    if (!tick_active) {
        ROS_WARN("Ending a tick that wasn't active.");
    }
    tick_active = false;
    ROS_INFO("Tick ended.");
}

bool moveTurtle(QPointF& pos, int& orientation) {
    if (!tick_active) {
        ROS_ERROR("moveTurtle called outside of an active tick.");
        return false;
    }

    if (action_done) {
        ROS_ERROR("Action already completed for this tick.");
        return false;
    }

    bool wall_detected = checkObstacle(pos, orientation);
    bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
                              static_cast<int>(std::floor(pos.y())));

    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);

    if (!next_move.validAction) {
        ROS_WARN("No valid action provided.");
        return false;
    }

    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected) {
                switch (orientation) {
                    case 0: pos.setX(pos.x() - 1.0); break; // WEST
                    case 1: pos.setY(pos.y() - 1.0); break; // NORTH
                    case 2: pos.setX(pos.x() + 1.0); break; // EAST
                    case 3: pos.setY(pos.y() + 1.0); break; // SOUTH
                    default:
                        ROS_ERROR("Invalid orientation in moveTurtle.");
                        return false;
                }
                displayVisits(next_move.visitCount);
            } else {
                ROS_WARN("Turtle attempted to move into a wall.");
            }
            break;

        case RIGHT:
            orientation = (orientation + 1) % 4;
            break;

        case LEFT:
            orientation = (orientation + 3) % 4;
            break;

        default:
            ROS_ERROR("Invalid action in moveTurtle.");
            return false;
    }

    action_done = true;
    return true;
}
