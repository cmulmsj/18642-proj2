/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/08/2024
 */

#ifdef testing
#include "student_mock.h"
#define ROS_INFO mock_ros_info
#define ROS_ERROR mock_ros_error
#else
#include "student.h"
#include "ros/ros.h"

// Global state variables
RobotState robot_state = STARTUP;
static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
static bool first_run = true;
static coordinate current_pos = {START_POS, START_POS};
static int facing_direction = 1; // Start facing NORTH
static int scans_completed = 0;  // Track how many directions we've scanned
static bool need_to_move = false;  // Whether we're ready to move after scanning
static int target_direction = -1;   // Direction we want to face for movement
static uint8_t min_visits = UINT8_MAX;  // Track least visited adjacent square

void updateVisitMap(coordinate pos) {
    if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
        visit_grid[pos.x][pos.y]++;
        ROS_INFO("Updated visit count at (%d,%d) to %d", pos.x, pos.y, visit_grid[pos.x][pos.y]);
    }
}

int getVisitCount(coordinate pos) {
    if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
        return visit_grid[pos.x][pos.y];
    }
    return INT_MAX;
}

coordinate getNextPosition(coordinate pos, int direction) {
    coordinate next = pos;
    switch (direction) {
        case 0: next.x--; break; // WEST
        case 1: next.y--; break; // NORTH
        case 2: next.x++; break; // EAST
        case 3: next.y++; break; // SOUTH
        default:
            ROS_ERROR("Invalid direction in getNextPosition");
            break;
    }
    return next;
}
#endif

turtleMove studentTurtleStep(bool bumped_wall, bool at_goal) {
    turtleMove next_move = {FORWARD, true, 0};
    
    // Stop if maze complete
    if (at_goal) {
        next_move.validAction = false;
        return next_move;
    }
    
    // Initialize on first run
    if (first_run) {
        updateVisitMap(current_pos);
        next_move.visitCount = static_cast<uint8_t>(
            std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
        );
        first_run = false;
        robot_state = PLAN_NEXT;
        scans_completed = 0;
        need_to_move = false;
        target_direction = -1;
        min_visits = UINT8_MAX;
        return next_move;
    }
    
    switch (robot_state) {
        case STARTUP: {
            robot_state = PLAN_NEXT;
            scans_completed = 0;
            need_to_move = false;
            target_direction = -1;
            min_visits = UINT8_MAX;
            next_move.validAction = false;
            break;
        }
        
        case PLAN_NEXT: {
            if (need_to_move) {
                // If we need to turn to face target direction
                if (facing_direction != target_direction) {
                    // Turn right (90 degrees per tick)
                    next_move.action = RIGHT;
                    facing_direction = (facing_direction + 1) % 4;
                } else {
                    // We're facing the right direction, try to move
                    if (!bumped_wall) {
                        next_move.action = FORWARD;
                        current_pos = getNextPosition(current_pos, facing_direction);
                        updateVisitMap(current_pos);
                        next_move.visitCount = static_cast<uint8_t>(
                            std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
                        );
                    }
                    // Reset scanning state after move attempt
                    need_to_move = false;
                    scans_completed = 0;
                    target_direction = -1;
                    min_visits = UINT8_MAX;
                }
            } else {
                // During scanning phase
                if (!bumped_wall) {
                    // Found potential path, check visit count
                    coordinate next_pos = getNextPosition(current_pos, facing_direction);
                    uint8_t visits = static_cast<uint8_t>(
                        std::min(getVisitCount(next_pos), static_cast<int>(UINT8_MAX))
                    );
                    if (visits < min_visits) {
                        min_visits = visits;
                        target_direction = facing_direction;
                    }
                }
                
                // Turn right to continue scanning (90 degrees per tick)
                next_move.action = RIGHT;
                facing_direction = (facing_direction + 1) % 4;
                scans_completed++;
                
                // After full scan, prepare to move if we found a path
                if (scans_completed >= 4) {
                    if (target_direction != -1) {
                        need_to_move = true;
                    } else {
                        // No valid path found, reset scan
                        scans_completed = 0;
                        min_visits = UINT8_MAX;
                    }
                }
            }
            break;
        }
        
        case MOVING: {
            robot_state = PLAN_NEXT;
            next_move.validAction = false;
            break;
        }
        
        default: {
            ROS_ERROR("Invalid robot state");
            next_move.validAction = false;
            break;
        }
    }
    
    ROS_INFO("Position: (%d,%d), Facing: %d, State: %d, Action: %d, Scans: %d, "
             "Target Dir: %d, Wall: %d, Need Move: %d", 
             current_pos.x, current_pos.y, facing_direction, robot_state, 
             next_move.action, scans_completed, target_direction, 
             bumped_wall, need_to_move);
    
    return next_move;
}



// #ifdef testing
// #include "student_mock.h"
// #define ROS_INFO mock_ros_info
// #define ROS_ERROR mock_ros_error
// #else
// #include "student.h"
// #include "ros/ros.h"

// // Global state variables
// RobotState robot_state = STARTUP;
// static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
// static bool first_run = true;
// static coordinate current_pos = {START_POS, START_POS};
// static int facing_direction = 1; // Start facing NORTH
// static int rotations_checked = 0;
// static uint8_t min_visits = UINT8_MAX;
// static int best_direction = -1;

// // Helper functions for tracking visited positions
// void updateVisitMap(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         visit_grid[pos.x][pos.y]++;
//         ROS_INFO("Updated visit count at (%d,%d) to %d", 
//                  pos.x, pos.y, visit_grid[pos.x][pos.y]);
//     }
// }

// int getVisitCount(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         return visit_grid[pos.x][pos.y];
//     }
//     return INT_MAX;
// }
// #endif

// // Get next position based on current position and direction
// coordinate getNextPosition(coordinate pos, int direction) {
//     coordinate next = pos;
//     switch (direction) {
//         case 0: next.x--; break; // WEST
//         case 1: next.y--; break; // NORTH
//         case 2: next.x++; break; // EAST
//         case 3: next.y++; break; // SOUTH
//         default:
//             ROS_ERROR("Invalid direction in getNextPosition");
//             break;
//     }
//     return next;
// }

// turtleMove studentTurtleStep(bool bumped_wall, bool at_goal) {
//     turtleMove next_move = {FORWARD, true, 0};
    
//     // Stop if maze complete
//     if (at_goal) {
//         next_move.validAction = false;
//         return next_move;
//     }
    
//     // Initialize on first run
//     if (first_run) {
//         updateVisitMap(current_pos);
//         next_move.visitCount = static_cast<uint8_t>(
//             std::min(getVisitCount(current_pos), 
//                     static_cast<int>(UINT8_MAX))
//         );
//         first_run = false;
//         robot_state = PLAN_NEXT;
//         rotations_checked = 0;
//         min_visits = UINT8_MAX;
//         best_direction = -1;
//         return next_move;
//     }
    
//     switch (robot_state) {
//         case STARTUP: {
//             robot_state = PLAN_NEXT;
//             rotations_checked = 0;
//             min_visits = UINT8_MAX;
//             best_direction = -1;
//             next_move.validAction = false;
//             break;
//         }
        
//         case PLAN_NEXT: {
//             // Check current direction first
//             if (rotations_checked == 0) {
//                 min_visits = UINT8_MAX;
//                 best_direction = -1;
//             }
            
//             // Evaluate current facing direction
//             if (!bumped_wall) {
//                 coordinate next_pos = getNextPosition(current_pos, facing_direction);
//                 uint8_t current_visits = static_cast<uint8_t>(
//                     std::min(getVisitCount(next_pos), 
//                             static_cast<int>(UINT8_MAX))
//                 );
//                 if (current_visits < min_visits) {
//                     min_visits = current_visits;
//                     best_direction = facing_direction;
//                 }
//             }
            
//             // Continue scanning directions or execute movement
//             if (rotations_checked < 3) {
//                 rotations_checked++;
//                 facing_direction = (facing_direction + 1) % 4;
//                 next_move.action = RIGHT;
//             } else {
//                 if (best_direction != facing_direction) {
//                     // Turn toward best direction
//                     int diff = (best_direction - facing_direction + 4) % 4;
//                     if (diff == 1) {
//                         facing_direction = (facing_direction + 1) % 4;
//                         next_move.action = RIGHT;
//                     } else {
//                         facing_direction = (facing_direction + 3) % 4;
//                         next_move.action = LEFT;
//                     }
//                 } else if (!bumped_wall) {
//                     // Move forward in best direction
//                     coordinate next_pos = getNextPosition(current_pos, facing_direction);
//                     current_pos = next_pos;
//                     updateVisitMap(current_pos);
//                     next_move.action = FORWARD;
//                     next_move.visitCount = static_cast<uint8_t>(
//                         std::min(getVisitCount(current_pos), 
//                                 static_cast<int>(UINT8_MAX))
//                     );
//                     // Reset for next planning cycle
//                     rotations_checked = 0;
//                     min_visits = UINT8_MAX;
//                     best_direction = -1;
//                 }
//             }
//             break;
//         }
        
//         case MOVING: {
//             robot_state = PLAN_NEXT;
//             next_move.validAction = false;
//             break;
//         }
        
//         default: {
//             ROS_ERROR("Invalid robot state");
//             next_move.validAction = false;
//             break;
//         }
//     }
    
//     // Log current state for debugging
//     ROS_INFO("Position: (%d,%d), Facing: %d, State: %d, Action: %d, "
//              "Rotations: %d, Best Dir: %d", 
//              current_pos.x, current_pos.y, facing_direction, 
//              robot_state, next_move.action, rotations_checked, 
//              best_direction);
    
//     return next_move;
// }
