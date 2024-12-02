/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/08/2024
 */

// #include "student.h"

// // Helper function to check obstacles in a given direction
// bool checkObstacle(QPointF pos, int direction) {
//     int x = static_cast<int>(std::floor(pos.x()));
//     int y = static_cast<int>(std::floor(pos.y()));
//     int x1 = x, y1 = y;
//     int x2 = x, y2 = y;

//     switch (direction) {
//         case 0: // WEST
//             y2 = y + 1;
//             break;
//         case 1: // NORTH
//             x2 = x + 1;
//             break;
//         case 2: // EAST
//             x1 = x + 1;
//             x2 = x + 1;
//             y2 = y + 1;
//             break;
//         case 3: // SOUTH
//             x2 = x + 1;
//             y1 = y + 1;
//             y2 = y + 1;
//             break;
//         default:
//             ROS_ERROR("Invalid direction in checkObstacle");
//             return true; // Treat invalid direction as obstacle
//     }
//     return bumped(x1, y1, x2, y2);
// }

// // Helper function to validate position and goal state
// bool checkAtEnd(const QPointF& pos) {
//     int x = static_cast<int>(std::floor(pos.x()));
//     int y = static_cast<int>(std::floor(pos.y()));
//     return atend(x, y);
// }

// bool moveTurtle(QPointF& pos, int& orientation) {
//     // State tracking for proper tick synchronization
//     static bool in_rotation = false;
//     static int target_orientation = orientation;
//     static bool move_pending = false;
//     static turtleMove pending_move = {FORWARD, true, 0};
    
//     // Fixed tick duration for timing
//     static const ros::Duration TICK_DURATION(0.2);
//     ros::Duration(TICK_DURATION).sleep();
    
//     // Get current state at start of tick
//     bool wall_detected = checkObstacle(pos, orientation);
//     bool reached_goal = checkAtEnd(pos);
    
//     // If in rotation, complete it before doing anything else
//     if (in_rotation) {
//         orientation = target_orientation;
//         in_rotation = false;
//         return true;
//     }
    
//     // If move is pending and we're clear to move, execute it
//     if (move_pending) {
//         if (!wall_detected && !reached_goal) {
//             switch (orientation) {
//                 case 0: pos.setX(pos.x() - 1.0); break;  // WEST
//                 case 1: pos.setY(pos.y() - 1.0); break;  // NORTH
//                 case 2: pos.setX(pos.x() + 1.0); break;  // EAST
//                 case 3: pos.setY(pos.y() + 1.0); break;  // SOUTH
//                 default:
//                     ROS_ERROR("Invalid orientation during movement");
//                     return false;
//             }
//             displayVisits(pending_move.visitCount);
//         }
//         move_pending = false;
//         return true;
//     }
    
//     // Get next move decision
//     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
//     if (!next_move.validAction) {
//         return false;
//     }
    
//     // Process the move based on action type
//     switch (next_move.action) {
//         case FORWARD:
//             if (!wall_detected && !reached_goal) {
//                 move_pending = true;
//                 pending_move = next_move;
//             }
//             break;
            
//         case RIGHT:
//         case LEFT:
//             in_rotation = true;
//             target_orientation = (orientation + (next_move.action == RIGHT ? 1 : 3)) % 4;
//             break;
            
//         default:
//             ROS_ERROR("Invalid action received");
//             return false;
//     }
    
//     return true;
// }


#include "student.h"

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
            ROS_ERROR("Invalid direction");
            return true;
    }
    return bumped(x1, y1, x2, y2);
}

bool moveTurtle(QPointF& pos, int& orientation) {
    static const ros::Duration TICK_DURATION(0.2);
    ros::Duration(TICK_DURATION).sleep();
    
    // Single set of state checks per tick
    bool wall_detected = checkObstacle(pos, orientation);
    bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
                             static_cast<int>(std::floor(pos.y())));
    
    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
    if (!next_move.validAction) {
        return false;
    }
    
    // Execute single action per tick
    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected && !reached_goal) {
                switch (orientation) {
                    case 0: pos.setX(pos.x() - 1.0); break;
                    case 1: pos.setY(pos.y() - 1.0); break;
                    case 2: pos.setX(pos.x() + 1.0); break;
                    case 3: pos.setY(pos.y() + 1.0); break;
                    default:
                        ROS_ERROR("Invalid orientation");
                        return false;
                }
                displayVisits(next_move.visitCount);
            }
            break;
            
        case RIGHT:
            orientation = (orientation + 1) % 4;
            break;
            
        case LEFT:
            orientation = (orientation + 3) % 4;
            break;
            
        default:
            ROS_ERROR("Invalid action");
            return false;
    }
    
    return true;
}

// #include "student.h"

// bool checkObstacle(QPointF pos, int direction) {
//     int x = static_cast<int>(std::floor(pos.x()));
//     int y = static_cast<int>(std::floor(pos.y()));
//     int x1 = x, y1 = y;
//     int x2 = x, y2 = y;

//     switch (direction) {
//         case 0: // WEST
//             y2 = y + 1;
//             break;
//         case 1: // NORTH
//             x2 = x + 1;
//             break;
//         case 2: // EAST
//             x1 = x + 1;
//             x2 = x + 1;
//             y2 = y + 1;
//             break;
//         case 3: // SOUTH
//             x2 = x + 1;
//             y1 = y + 1;
//             y2 = y + 1;
//             break;
//         default:
//             ROS_ERROR("Invalid direction in checkObstacle");
//             return true; // Treat invalid direction as obstacle
//     }
//     return bumped(x1, y1, x2, y2);
// }

// // bool moveTurtle(QPointF& pos, int& orientation) {
// //     // Add delay for timeout
// //     static const double MOVE_DELAY = 0.2;
// //     ros::Duration(MOVE_DELAY).sleep();  // 200ms delay
    
// //     // Check for wall and goal
// //     bool wall_detected = checkObstacle(pos, orientation);
// //     bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
// //                              static_cast<int>(std::floor(pos.y())));
    
// //     // Get next move
// //     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
// //     if (!next_move.validAction) {
// //         return false;
// //     }
    
// //     // Execute move
// //     switch (next_move.action) {
// //         case FORWARD:
// //             if (!wall_detected) {
// //                 switch (orientation) {
// //                     case 0: pos.setX(pos.x() - 1.0); break;
// //                     case 1: pos.setY(pos.y() - 1.0); break;
// //                     case 2: pos.setX(pos.x() + 1.0); break;
// //                     case 3: pos.setY(pos.y() + 1.0); break;
// //                     default:
// //                         ROS_ERROR("Invalid orientation in moveTurtle");
// //                         return false;
// //                 }
// //                 displayVisits(next_move.visitCount);
// //             }
// //             break;
            
// //         case RIGHT:
// //             orientation = (orientation + 1) % 4;
// //             break;
            
// //         case LEFT:
// //             orientation = (orientation + 3) % 4;
// //             break;
            
// //         default:
// //             ROS_ERROR("Invalid action in moveTurtle");
// //             return false;
// //     }
    
// //     return true;
// // }

// bool moveTurtle(QPointF& pos, int& orientation) {
//     // Start new tick cycle
//     static const ros::Duration TICK_DURATION(0.2);
//     ros::Duration(TICK_DURATION).sleep();
    
//     // Get current state first before any actions
//     bool wall_detected = checkObstacle(pos, orientation);
//     bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
//                              static_cast<int>(std::floor(pos.y())));
    
//     // Get next move within this tick
//     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
//     if (!next_move.validAction) {
//         return false;
//     }

//     // Handle rotation and movement in separate ticks
//     static bool in_rotation = false;
//     static int target_orientation = orientation;
    
//     if (in_rotation) {
//         // Complete rotation before allowing movement
//         orientation = target_orientation;
//         in_rotation = false;
//         return true;
//     }
    
//     // Execute move based on action type
//     switch (next_move.action) {
//         case FORWARD:
//             if (!wall_detected && !reached_goal && !in_rotation) {
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
//             if (!in_rotation) {
//                 in_rotation = true;
//                 target_orientation = (orientation + (next_move.action == RIGHT ? 1 : 3)) % 4;
//             }
//             break;
            
//         default:
//             ROS_ERROR("Invalid action");
//             return false;
//     }
    
//     return true;
// }

