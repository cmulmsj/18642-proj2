// /* 
//  * Originally by Philip Koopman (koopman@cmu.edu)
//  * and Milda Zizyte (milda@cmu.edu)
//  *
//  * STUDENT NAME: Mashengjun Li
//  * ANDREW ID: mashengl
//  * LAST UPDATE: 12/02/2024
//  */

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
//             ROS_ERROR("Invalid direction");
//             return true;
//     }
//     return bumped(x1, y1, x2, y2);
// }

// bool moveTurtle(QPointF& pos, int& orientation) {
//     static const ros::Duration TICK_DURATION(0.5);
//     ros::Duration(TICK_DURATION).sleep();
    
//     // Single set of state checks per tick
//     bool wall_detected = checkObstacle(pos, orientation);
//     bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
//                              static_cast<int>(std::floor(pos.y())));
    
//     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
//     if (!next_move.validAction) {
//         return false;
//     }
    
//     // Execute single action per tick
//     switch (next_move.action) {
//         case FORWARD:
//             if (!wall_detected && !reached_goal) {
//                 switch (orientation) {
//                     case 0: pos.setX(pos.x() - 1.0); break;
//                     case 1: pos.setY(pos.y() - 1.0); break;
//                     case 2: pos.setX(pos.x() + 1.0); break;
//                     case 3: pos.setY(pos.y() + 1.0); break;
//                     default:
//                         ROS_ERROR("Invalid orientation");
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
//             ROS_ERROR("Invalid action");
//             return false;
//     }
    
//     return true;
// }


// /* 
//  * Originally by Philip Koopman (koopman@cmu.edu)
//  * and Milda Zizyte (milda@cmu.edu)
//  *
//  * STUDENT NAME: Mashengjun Li
//  * ANDREW ID: mashengl
//  * LAST UPDATE: 12/02/2024
//  */

// #include "student.h"

// // Tick state management
// static struct TickState {
//     bool is_active;
//     int countdown;
//     bool move_completed;
// } tick_state = {false, 0, false};

// bool checkObstacle(QPointF pos, int direction) {
//     if (!tick_state.is_active) {
//         return false;
//     }

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
//             ROS_ERROR("Invalid direction");
//             return true;
//     }
//     return bumped(x1, y1, x2, y2);
// }

// bool moveTurtle(QPointF& pos, int& orientation) {
//     static const ros::Duration TICK_SLEEP(0.015);
//     static const ros::Duration MSG_WAIT(0.002);
    
//     // Handle countdown
//     if (tick_state.countdown > 0) {
//         tick_state.countdown--;
//         return true;
//     }
    
//     // Sleep before starting new tick
//     TICK_SLEEP.sleep();
    
//     // Start new tick
//     tick_state.is_active = true;
//     tick_state.move_completed = false;
    
//     // Get state checks inside active tick
//     bool wall_detected = checkObstacle(pos, orientation);
//     bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
//                              static_cast<int>(std::floor(pos.y())));
    
//     if (reached_goal) {
//         // Wait for messages
//         MSG_WAIT.sleep();
//         tick_state.is_active = false;
//         return false;
//     }
    
//     turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    
//     if (!next_move.validAction) {
//         // Wait for messages
//         MSG_WAIT.sleep();
//         tick_state.is_active = false;
//         return false;
//     }
    
//     // Execute single action within tick
//     switch (next_move.action) {
//         case FORWARD:
//             if (!wall_detected && !reached_goal) {
//                 switch (orientation) {
//                     case 0: pos.setX(pos.x() - 1.0); break;
//                     case 1: pos.setY(pos.y() - 1.0); break;
//                     case 2: pos.setX(pos.x() + 1.0); break;
//                     case 3: pos.setY(pos.y() + 1.0); break;
//                     default:
//                         ROS_ERROR("Invalid orientation");
//                         tick_state.is_active = false;
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
//             ROS_ERROR("Invalid action");
//             tick_state.is_active = false;
//             return false;
//     }
    
//     // Wait for messages before ending tick
//     MSG_WAIT.sleep();
    
//     // Keep tick active slightly longer
//     tick_state.countdown = TIMEOUT;
//     tick_state.move_completed = true;
//     tick_state.is_active = false;
    
//     return true;
// }

/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 12/02/2024
 */

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
    static bool first_move = true;
    static int cycles = 0;
    bool state_changed = false;

    if (first_move) {
        // Ensure the turtle starts at the correct known position
        // If the monitor expects the turtle at (0,0) facing West (0), do that:
        pos.setX(0.0);
        pos.setY(0.0);
        orientation = 0; // WEST if that's what orientation 0 means
        first_move = false;
        cycles = TIMEOUT;
        return true; 
    }

    // If waiting out cycles, no action taken
    if (cycles > 0) {
        cycles--;
        return false;
    }

    // Compute current integer coordinates from pos
    int current_x = static_cast<int>(std::floor(pos.x()));
    int current_y = static_cast<int>(std::floor(pos.y()));

    // Check if at goal BEFORE moving
    bool reached_goal = atend(current_x, current_y);
    if (reached_goal) {
        return false; // Turtle is at the end, no move needed
    }

    // Check for walls in the direction the turtle is facing
    bool wall_detected = checkObstacle(pos, orientation);

    // Now decide next action
    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    if (!next_move.validAction) {
        // No action means no state change
        return false;
    }

    // Execute the move
    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected && !reached_goal) {
                // Move the turtle according to orientation
                switch (orientation) {
                    case 0: // WEST
                        pos.setX(pos.x() - 1.0);
                        break;
                    case 1: // NORTH
                        pos.setY(pos.y() - 1.0);
                        break;
                    case 2: // EAST
                        pos.setX(pos.x() + 1.0);
                        break;
                    case 3: // SOUTH
                        pos.setY(pos.y() + 1.0);
                        break;
                    default:
                        ROS_ERROR("Invalid orientation");
                        return false;
                }
                // After moving, update visits
                displayVisits(next_move.visitCount);
                state_changed = true;
            }
            break;

        case RIGHT:
            orientation = (orientation + 1) % 4;
            state_changed = true;
            break;

        case LEFT:
            orientation = (orientation + 3) % 4;
            state_changed = true;
            break;

        default:
            ROS_ERROR("Invalid action");
            return false;
    }

    cycles = TIMEOUT;
    return state_changed;
}



// bool moveTurtle(QPointF& pos, int& orientation) {
//     static const ros::Duration TICK_DURATION(0.5);

//     // Wait for previous tick's messages to be processed
//     ros::Duration(TICK_DURATION * 0.5).sleep();
    
//     // Start new tick
//     tick_state.is_active = true;
//     tick_state.needs_check = true;
    
//     // Perform all state checks at tick start
//     tick_state.wall_detected = checkObstacle(pos, orientation);
//     tick_state.goal_reached = atend(static_cast<int>(std::floor(pos.x())), 
//                                   static_cast<int>(std::floor(pos.y())));
    
//     turtleMove next_move = studentTurtleStep(tick_state.wall_detected, 
//                                            tick_state.goal_reached);
    
//     if (!next_move.validAction) {
//         tick_state.is_active = false;
//         return false;
//     }
    
//     // Execute single action
//     switch (next_move.action) {
//         case FORWARD:
//             if (!tick_state.wall_detected && !tick_state.goal_reached) {
//                 switch (orientation) {
//                     case 0: pos.setX(pos.x() - 1.0); break;
//                     case 1: pos.setY(pos.y() - 1.0); break;
//                     case 2: pos.setX(pos.x() + 1.0); break;
//                     case 3: pos.setY(pos.y() + 1.0); break;
//                     default:
//                         ROS_ERROR("Invalid orientation");
//                         tick_state.is_active = false;
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
//             ROS_ERROR("Invalid action");
//             tick_state.is_active = false;
//             return false;
//     }
    
//     // Allow time for action messages to be processed within tick
//     ros::Duration(TICK_DURATION * 0.5).sleep();
    
//     // End tick
//     tick_state.is_active = false;
//     return true;
// }
