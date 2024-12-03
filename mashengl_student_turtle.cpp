// /*
//  * Originally by Philip Koopman (koopman@cmu.edu)
//  * and Milda Zizyte (milda@cmu.edu)
//  *
//  * STUDENT NAME: Mashengjun Li
//  * ANDREW ID: mashengl
//  * LAST UPDATE: 12/02/2024
//  */

// #ifdef testing
// #include "student_mock.h"
// #define ROS_INFO mock_ros_info
// #define ROS_ERROR mock_ros_error
// #else
// #include "student.h"
// #include "ros/ros.h"

// // Global state variables
// static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
// static bool first_run = true;
// static coordinate current_pos = {START_POS, START_POS};
// static int facing_direction = 1; // Start facing NORTH
// static int scans_completed = 0;
// static bool ready_to_move = false;
// static int best_direction = -1;
// static uint8_t min_visit_count = UINT8_MAX;

// void updateVisitMap(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         visit_grid[pos.x][pos.y]++;
//     }
// }

// int getVisitCount(coordinate pos) {
//     if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
//         return visit_grid[pos.x][pos.y];
//     }
//     return INT_MAX;
// }

// coordinate getNextPosition(coordinate pos, int direction) {
//     coordinate next = pos;
//     switch (direction) {
//         case 0: next.x--; break; // WEST
//         case 1: next.y--; break; // NORTH
//         case 2: next.x++; break; // EAST
//         case 3: next.y++; break; // SOUTH
//         default:
//             ROS_ERROR("Invalid direction");
//             break;
//     }
//     return next;
// }
// #endif

// turtleMove studentTurtleStep(bool bumped_wall, bool at_goal) {
//     turtleMove next_move = {FORWARD, true, 0};
    
//     if (at_goal) {
//         next_move.validAction = false;
//         return next_move;
//     }
    
//     if (first_run) {
//         updateVisitMap(current_pos);
//         next_move.visitCount = static_cast<uint8_t>(
//             std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
//         );
//         first_run = false;
//         scans_completed = 0;
//         ready_to_move = false;
//         best_direction = -1;
//         min_visit_count = UINT8_MAX;
//         return next_move;
//     }
    
//     if (ready_to_move) {
//         if (facing_direction != best_direction) {
//             next_move.action = RIGHT;
//             facing_direction = (facing_direction + 1) % 4;
//         } else {
//             if (!bumped_wall) {
//                 next_move.action = FORWARD;
//                 current_pos = getNextPosition(current_pos, facing_direction);
//                 updateVisitMap(current_pos);
//                 next_move.visitCount = static_cast<uint8_t>(
//                     std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
//                 );
//             }
//             ready_to_move = false;
//             scans_completed = 0;
//             best_direction = -1;
//             min_visit_count = UINT8_MAX;
//         }
//     } else {
//         if (!bumped_wall) {
//             coordinate next_pos = getNextPosition(current_pos, facing_direction);
//             uint8_t visit_count = static_cast<uint8_t>(
//                 std::min(getVisitCount(next_pos), static_cast<int>(UINT8_MAX))
//             );
//             if (visit_count < min_visit_count) {
//                 min_visit_count = visit_count;
//                 best_direction = facing_direction;
//             }
//         }
        
//         next_move.action = RIGHT;
//         facing_direction = (facing_direction + 1) % 4;
//         scans_completed++;
        
//         if (scans_completed >= 4) {
//             if (best_direction != -1) {
//                 ready_to_move = true;
//             } else {
//                 scans_completed = 0;
//                 min_visit_count = UINT8_MAX;
//             }
//         }
//     }
    
//     return next_move;
// }

/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 12/02/2024
 */

#ifdef testing
#include "student_mock.h"
#define ROS_INFO mock_ros_info
#define ROS_ERROR mock_ros_error
#else
#include "student.h"
#include "ros/ros.h"

// Global state variables
static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
static bool first_run = true;
static coordinate current_pos = {START_POS, START_POS};
static int facing_direction = 1; // Start facing NORTH
static int scan_count = 0;
static int best_direction = -1;
static uint8_t min_visit_count = UINT8_MAX;

void updateVisitMap(coordinate pos) {
    if (pos.x < GRID_SIZE && pos.y < GRID_SIZE) {
        visit_grid[pos.x][pos.y]++;
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
            ROS_ERROR("Invalid direction");
            break;
    }
    return next;
}
#endif

turtleMove studentTurtleStep(bool bumped_wall, bool at_goal) {
    turtleMove next_move = {FORWARD, true, 0};
    
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
        return next_move;
    }
    
    // Scanning phase - limited to 4 scans
    if (scan_count < 4) {
        coordinate next_pos = getNextPosition(current_pos, facing_direction);
        uint8_t visit_count = static_cast<uint8_t>(
            std::min(getVisitCount(next_pos), static_cast<int>(UINT8_MAX))
        );
        
        if (!bumped_wall && visit_count < min_visit_count) {
            min_visit_count = visit_count;
            best_direction = facing_direction;
        }
        
        next_move.action = RIGHT;
        facing_direction = (facing_direction + 1) % 4;
        scan_count++;
    }
    // Movement phase - after scanning complete
    else {
        if (best_direction == -1) {
            // No valid direction found, reset scan
            scan_count = 0;
            min_visit_count = UINT8_MAX;
            next_move.action = RIGHT;
        }
        else if (facing_direction != best_direction) {
            // Turn to face best direction
            next_move.action = RIGHT;
            facing_direction = (facing_direction + 1) % 4;
        }
        else {
            // Move forward in best direction
            if (!bumped_wall) {
                next_move.action = FORWARD;
                current_pos = getNextPosition(current_pos, facing_direction);
                updateVisitMap(current_pos);
                
                // Reset for next scan
                scan_count = 0;
                best_direction = -1;
                min_visit_count = UINT8_MAX;
            }
        }
    }
    
    // Always update the visit count in the move
    next_move.visitCount = static_cast<uint8_t>(
        std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
    );
    
    return next_move;
}
