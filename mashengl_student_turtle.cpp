/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/08/2024
 */

// #ifdef testing
// #include "student_mock.h"
// #define ROS_INFO mock_ros_info
// #define ROS_ERROR mock_ros_error
// #else
// #include "student.h"
// #include "ros/ros.h"

// // These are only needed in normal operation mode
// RobotState robot_state = STARTUP;
// static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
// static bool first_run = true;
// static coordinate current_pos = {START_POS, START_POS};
// static int facing_direction = 1; // Start facing NORTH
// static int rotations_checked = 0;
// static uint8_t min_visits = UINT8_MAX;
// static int best_direction = -1;

// void updateVisitMap(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         visit_grid[pos.x][pos.y]++;
//         ROS_INFO("Updated visit count at (%d,%d) to %d", pos.x, pos.y, visit_grid[pos.x][pos.y]);
//     }
// }

// int getVisitCount(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         return visit_grid[pos.x][pos.y];
//     }
//     return INT_MAX;
// }
// #endif

// // Common code for both modes
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
//         // Safe conversion using min to avoid overflow
//         next_move.visitCount = static_cast<uint8_t>(
//             std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
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

//             // Check current facing direction
//             if (!bumped_wall) {
//                 coordinate next_pos = getNextPosition(current_pos, facing_direction);
//                 // Safe conversion using min
//                 uint8_t current_visits = static_cast<uint8_t>(
//                     std::min(getVisitCount(next_pos), static_cast<int>(UINT8_MAX))
//                 );
//                 if (current_visits < min_visits) {
//                     min_visits = current_visits;
//                     best_direction = facing_direction;
//                 }
//             }

//             if (rotations_checked < 3) {
//                 // Continue scanning all directions
//                 rotations_checked++;
//                 facing_direction = (facing_direction + 1) % 4;
//                 next_move.action = RIGHT;
//             } else {
//                 // We've checked all directions, move in best direction
//                 if (best_direction != facing_direction) {
//                     // Turn toward best direction using shortest path
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
//                     // Safe conversion using min
//                     next_move.visitCount = static_cast<uint8_t>(
//                         std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
//                     );
//                     // Reset for next scan
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

//     ROS_INFO("Position: (%d,%d), Facing: %d, State: %d, Action: %d, Rotations: %d, Best Dir: %d", 
//              current_pos.x, current_pos.y, facing_direction, robot_state, 
//              next_move.action, rotations_checked, best_direction);

//     return next_move;
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
//             std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
//         );
//         first_run = false;
//         robot_state = PLAN_NEXT;
//         rotations_checked = 0;
//         return next_move;
//     }

//     // Key decision logic
//     if (bumped_wall || rotations_checked < 4) {
//         // If we hit a wall or haven't checked all directions, rotate
//         rotations_checked++;
//         next_move.action = RIGHT;
        
//         // After checking all directions, reset counter
//         if (rotations_checked >= 4) {
//             rotations_checked = 0;
//         }
//     } else {
//         // No wall ahead and we've checked all directions, move forward
//         coordinate next_pos = getNextPosition(current_pos, facing_direction);
//         current_pos = next_pos;
//         updateVisitMap(current_pos);
//         next_move.action = FORWARD;
//         next_move.visitCount = static_cast<uint8_t>(
//             std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
//         );
//         rotations_checked = 0;  // Reset rotation counter after moving
//     }

//     return next_move;
// }

/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 */

#ifdef testing
#include "student_mock.h"
#define ROS_INFO mock_ros_info
#define ROS_ERROR mock_ros_error
#else
#include "student.h"
#include "ros/ros.h"
#endif

// Global state variables
static RobotState current_state = S1_BEGIN;
static uint8_t visit_map[GRID_SIZE][GRID_SIZE] = {{0}};
static coordinate pos = {START_POS, START_POS};
static int orientation = WEST;
static int check_count = 0;
static bool walls[4] = {false};  // Track walls in each direction
static uint8_t min_visits = UINT8_MAX;
static int best_dir = -1;
static bool is_initialized = false;

void update_visits(coordinate p) {
    if (p.x < GRID_SIZE && p.y < GRID_SIZE) {
        visit_map[p.x][p.y]++;
    }
}

coordinate next_position(int dir) {
    coordinate next = pos;
    switch (dir) {
        case WEST:  next.x--; break;
        case NORTH: next.y--; break;
        case EAST:  next.x++; break;
        case SOUTH: next.y++; break;
        default:
            ROS_ERROR("Invalid direction in next_position");
            break;
    }
    return next;
}

bool is_valid_position(coordinate next) {
    return next.x < GRID_SIZE && next.y < GRID_SIZE;
}

turtleMove studentTurtleStep(bool wall_detected, bool at_goal) {
    turtleMove result = {FORWARD, true, 0};

    // Handle initialization
    if (!is_initialized) {
        update_visits(pos);
        result.visitCount = visit_map[pos.x][pos.y];
        is_initialized = true;
        current_state = S1_BEGIN;
        return result;
    }

    // Handle completion
    if (at_goal) {
        current_state = S3_END;
        result.validAction = false;
        return result;
    }

    switch (current_state) {
        case S1_BEGIN:
            check_count = 0;
            min_visits = UINT8_MAX;
            best_dir = -1;
            for (int i = 0; i < 4; i++) walls[i] = false;
            current_state = S4_BUMPED_1;
            result.validAction = false;
            break;

        case S4_BUMPED_1:
        case S5_BUMPED_2:
        case S6_BUMPED_3:
        case S7_BUMPED_4:
            // Store wall information for current direction
            walls[orientation] = wall_detected;
            
            // Check visit count in this direction if no wall
            if (!wall_detected) {
                coordinate next = next_position(orientation);
                if (is_valid_position(next)) {
                    if (visit_map[next.x][next.y] < min_visits) {
                        min_visits = visit_map[next.x][next.y];
                        best_dir = orientation;
                    }
                }
            }
            
            // Continue checking other directions
            result.action = RIGHT;
            orientation = (orientation + 1) % 4;
            current_state = (RobotState)(((int)current_state) + 1);
            if (current_state > S7_BUMPED_4) {
                current_state = S8_MOVE_1;
                result.validAction = false;
            }
            break;

        case S8_MOVE_1:
            if (best_dir == -1) {
                // No valid direction found, start over
                current_state = S1_BEGIN;
                result.validAction = false;
            } else if (best_dir != orientation) {
                // Need to rotate to best direction
                result.action = RIGHT;
                orientation = (orientation + 1) % 4;
                current_state = S9_MOVE_2;
            } else {
                current_state = S10_MOVE_3;
                result.validAction = false;
            }
            break;

        case S9_MOVE_2:
            if (!wall_detected) {
                current_state = S10_MOVE_3;
                result.validAction = false;
            } else {
                current_state = S1_BEGIN;
                result.validAction = false;
            }
            break;

        case S10_MOVE_3:
            if (!wall_detected) {
                // Move forward
                pos = next_position(orientation);
                update_visits(pos);
                result.action = FORWARD;
                result.visitCount = visit_map[pos.x][pos.y];
                current_state = S1_BEGIN;  // Prepare for next exploration
            } else {
                // Unexpected wall, restart exploration
                current_state = S1_BEGIN;
                result.validAction = false;
            }
            break;

        case S2_CHECK_END:
        case S3_END:
            result.validAction = false;
            break;

        default:
            ROS_ERROR("Invalid state");
            current_state = S1_BEGIN;
            result.validAction = false;
            break;
    }

    return result;
}
