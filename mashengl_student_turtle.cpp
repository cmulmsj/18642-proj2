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

// These are only needed in normal operation mode
RobotState robot_state = STARTUP;
static int visit_grid[GRID_SIZE][GRID_SIZE] = {{0}};
static bool first_run = true;
static coordinate current_pos = {START_POS, START_POS};
static int facing_direction = 1; // Start facing NORTH
static int rotations_checked = 0;
static uint8_t min_visits = UINT8_MAX;
static int best_direction = -1;

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
#endif

// Common code for both modes
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
        rotations_checked = 0;
        return next_move;
    }

    // Key decision logic
    if (bumped_wall || rotations_checked < 4) {
        // If we hit a wall or haven't checked all directions, rotate
        rotations_checked++;
        next_move.action = RIGHT;
        
        // After checking all directions, reset counter
        if (rotations_checked >= 4) {
            rotations_checked = 0;
        }
    } else {
        // No wall ahead and we've checked all directions, move forward
        coordinate next_pos = getNextPosition(current_pos, facing_direction);
        current_pos = next_pos;
        updateVisitMap(current_pos);
        next_move.action = FORWARD;
        next_move.visitCount = static_cast<uint8_t>(
            std::min(getVisitCount(current_pos), static_cast<int>(UINT8_MAX))
        );
        rotations_checked = 0;  // Reset rotation counter after moving
    }

    return next_move;
}
