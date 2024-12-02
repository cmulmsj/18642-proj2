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

bool moveTurtle(QPointF& pos, int& orientation) {
    // Ensure function is only called during an active tick
    if (!tick_active) {
        ROS_WARN("moveTurtle called outside of active tick.");
        return false;
    }

    // Ensure only one action is taken per tick
    if (action_done) {
        ROS_WARN("Action already completed for this tick.");
        return false;
    }

    // Check for obstacles and goal state
    bool wall_detected = checkObstacle(pos, orientation);
    bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
                             static_cast<int>(std::floor(pos.y())));

    // Get the next move from the turtle's decision-making logic
    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);

    if (!next_move.validAction) {
        ROS_WARN("No valid action returned by studentTurtleStep.");
        return false;
    }

    // Execute the move
    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected && !reached_goal) {
                switch (orientation) {
                    case 0: pos.setX(pos.x() - 1.0); break;  // WEST
                    case 1: pos.setY(pos.y() - 1.0); break;  // NORTH
                    case 2: pos.setX(pos.x() + 1.0); break;  // EAST
                    case 3: pos.setY(pos.y() + 1.0); break;  // SOUTH
                    default:
                        ROS_ERROR("Invalid orientation during move.");
                        return false;
                }
                displayVisits(next_move.visitCount);
                ROS_INFO("Turtle moved FORWARD to position (%.1f, %.1f).",
                         pos.x(), pos.y());
                action_done = true;  // Mark action as done for this tick
            } else {
                ROS_WARN("Turtle attempted to move forward but was blocked.");
            }
            break;

        case RIGHT:
            orientation = (orientation + 1) % 4;
            ROS_INFO("Turtle turned RIGHT. New orientation: %d", orientation);
            action_done = true;
            break;

        case LEFT:
            orientation = (orientation + 3) % 4;
            ROS_INFO("Turtle turned LEFT. New orientation: %d", orientation);
            action_done = true;
            break;

        default:
            ROS_ERROR("Invalid action returned by studentTurtleStep.");
            return false;
    }

    return true;
}
// Called at the start of each tick
void startTick() {
    tick_active = true;
    action_done = false;  // Reset for the new tick
}

// Called at the end of each tick
void endTick() {
    tick_active = false;
}


// bool moveTurtle(QPointF& pos, int& orientation) {
//     // Fixed delay for timing
//     ros::Duration(0.2).sleep();
    
//     // Check for wall and goal first
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
//             if (!wall_detected && !reached_goal) {
//                 switch (orientation) {
//                     case 0: pos.setX(pos.x() - 1.0); break;  // WEST
//                     case 1: pos.setY(pos.y() - 1.0); break;  // NORTH
//                     case 2: pos.setX(pos.x() + 1.0); break;  // EAST
//                     case 3: pos.setY(pos.y() + 1.0); break;  // SOUTH
//                     default:
//                         ROS_ERROR("Invalid orientation");
//                         return false;
//                 }
//                 displayVisits(next_move.visitCount);
//             }
//             break;
            
//         case RIGHT:
//         case LEFT:
//             // For turns, just update orientation
//             orientation = (orientation + (next_move.action == RIGHT ? 1 : 3)) % 4;
//             break;
            
//         default:
//             ROS_ERROR("Invalid action");
//             return false;
//     }
    
//     return true;
// }
