/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/01/2024
 */

#include "student.h"

// State machine variables
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

coordinate getNextPosition(coordinate pos, int direction) {
    coordinate next = pos;
    switch (direction) {
        case 0: next.x--; break; // WEST
        case 1: next.y--; break; // NORTH
        case 2: next.x++; break; // EAST
        case 3: next.y++; break; // SOUTH
    }
    return next;
}

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
        next_move.visitCount = getVisitCount(current_pos);
        first_run = false;
        robot_state = PLAN_NEXT;
        rotations_checked = 0;
        min_visits = UINT8_MAX;
        best_direction = -1;
        return next_move;
    }

    switch (robot_state) {
        case STARTUP: {
            robot_state = PLAN_NEXT;
            rotations_checked = 0;
            min_visits = UINT8_MAX;
            best_direction = -1;
            next_move.validAction = false;
            break;
        }

        case PLAN_NEXT: {
            // Check current direction first
            if (rotations_checked == 0) {
                min_visits = UINT8_MAX;
                best_direction = -1;
            }

            // Check current facing direction
            if (!bumped_wall) {
                coordinate next_pos = getNextPosition(current_pos, facing_direction);
                uint8_t current_visits = getVisitCount(next_pos);
                if (current_visits < min_visits) {
                    min_visits = current_visits;
                    best_direction = facing_direction;
                }
            }

            if (rotations_checked < 3) {
                // Continue scanning all directions
                rotations_checked++;
                facing_direction = (facing_direction + 1) % 4;
                next_move.action = RIGHT;
            } else {
                // We've checked all directions, move in best direction
                if (best_direction != facing_direction) {
                    // Turn toward best direction using shortest path
                    int diff = (best_direction - facing_direction + 4) % 4;
                    if (diff == 1) {
                        facing_direction = (facing_direction + 1) % 4;
                        next_move.action = RIGHT;
                    } else {
                        facing_direction = (facing_direction + 3) % 4;
                        next_move.action = LEFT;
                    }
                } else if (!bumped_wall) {
                    // Move forward in best direction
                    coordinate next_pos = getNextPosition(current_pos, facing_direction);
                    current_pos = next_pos;
                    updateVisitMap(current_pos);
                    next_move.action = FORWARD;
                    next_move.visitCount = getVisitCount(current_pos);
                    // Reset for next scan
                    rotations_checked = 0;
                    min_visits = UINT8_MAX;
                    best_direction = -1;
                }
            }
            break;
        }

        case MOVING: {
            robot_state = PLAN_NEXT;
            next_move.validAction = false;
            break;
        }
    }

    ROS_INFO("Position: (%d,%d), Facing: %d, State: %d, Action: %d, Rotations: %d, Best Dir: %d", 
             current_pos.x, current_pos.y, facing_direction, robot_state, 
             next_move.action, rotations_checked, best_direction);

    return next_move;
}

// #include "stdint.h"
// #include "student.h"

// static uint8_t visit_count_map[30][30] = {{0}};

// enum FSM_STATES { 
//     STATE_FORWARD = 0,           
//     STATE_UNVISITED = 1,   
//     STATE_UNBUMPED = 2  
// };

// enum LOCAL_DIRECTION { 
//     L_WEST = 0, 
//     L_NORTH = 1, 
//     L_EAST = 2, 
//     L_SOUTH = 3 
// };

// /**
//  * @brief Get number of visits for a cell
//  * @param local_coord Grid position to check
//  * @return Visit count for the specified cell
//  */
// uint8_t getVisit(coordinate local_coord) { 
//     uint8_t visit_count = visit_count_map[local_coord.x][local_coord.y];
//     ROS_INFO("TURTLE: Getting visit count at (%d,%d): %d", 
//              local_coord.x, local_coord.y, visit_count);
//     return visit_count;
// }

// /**
//  * @brief Update visit count for a cell
//  * @param local_coord Grid position to update
//  * @param setVal New visit count
//  */
// void setVisit(coordinate local_coord, uint8_t setVal) {
//     ROS_INFO("TURTLE: Updating visit count at (%d,%d): %d -> %d", 
//              local_coord.x, local_coord.y, visit_count_map[local_coord.x][local_coord.y], setVal);
//     visit_count_map[local_coord.x][local_coord.y] = setVal;
// }

// /**
//  * @brief Calculate next grid position based on current position and direction
//  * @param current_location Current position
//  * @param local_orientation Current direction
//  * @return New position after movement
//  */
// coordinate updateLocalTurtlePosition(coordinate current_location, LOCAL_DIRECTION local_orientation) {
//     coordinate new_location = current_location;
//     ROS_INFO("TURTLE: Current position (%d,%d), Direction: %d", 
//              current_location.x, current_location.y, local_orientation);

//     switch (local_orientation) {
//         case L_NORTH: {
//             new_location.y = static_cast<uint8_t>(new_location.y - static_cast<uint8_t>(1));
//             ROS_INFO("TURTLE: Moving NORTH");
//             break;
//         }
//         case L_EAST: {
//             new_location.x = static_cast<uint8_t>(new_location.x + static_cast<uint8_t>(1));
//             ROS_INFO("TURTLE: Moving EAST");
//             break;
//         }
//         case L_SOUTH: {
//             new_location.y = static_cast<uint8_t>(new_location.y + static_cast<uint8_t>(1));
//             ROS_INFO("TURTLE: Moving SOUTH");
//             break;
//         }
//         case L_WEST: {
//             new_location.x = static_cast<uint8_t>(new_location.x - static_cast<uint8_t>(1));
//             ROS_INFO("TURTLE: Moving WEST");
//             break;
//         }
//         default: {
//             ROS_ERROR("TURTLE: Invalid direction in updateLocalTurtlePosition: %d", local_orientation);
//             break;
//         }
//     }
    
//     ROS_INFO("TURTLE: New position: (%d,%d)", new_location.x, new_location.y);
//     return new_location;
// }

// /**
//  * @brief Main navigation algorithm implementation
//  * 
//  * Implements a finite state machine with three states:
//  * 1. STATE_FORWARD: Moving through unvisited cells
//  * 2. STATE_UNVISITED: Looking for unvisited paths when stuck
//  * 3. STATE_UNBUMPED: Finding optimal path when surrounded by visited cells
//  * 
//  * @param bumpedIntoWall True if wall detected ahead
//  * @param at_end True if goal reached
//  * @return Next movement command
//  */
// turtleMove studentTurtleStep(bool bumpedIntoWall, bool at_end) {
//     // Initialize static variables
//     static coordinate current_location;
//     static LOCAL_DIRECTION current_local_direction = L_NORTH;
//     static uint8_t directionsChecked = 0;
//     static FSM_STATES current_state = STATE_FORWARD;
//     static bool bumpedMap[4] = {false};
//     static bool first_run = true;

//     if (first_run) {
//         current_location.x = 14;
//         current_location.y = 14;
//         first_run = false;
//         ROS_INFO("TURTLE: Initialized starting position to (14,14)");
//     }

//     turtleMove futureMove;
//     const uint8_t TIMEOUT = 0.5;
//     static uint8_t timeout_counter;

//     ROS_INFO("\nTURTLE: === New Navigation Step ===");
//     ROS_INFO("TURTLE: Wall detected: %s, At goal: %s", 
//              bumpedIntoWall ? "YES" : "NO", at_end ? "YES" : "NO");
//     ROS_INFO("TURTLE: Current State: %d, Directions Checked: %d", 
//              current_state, directionsChecked);

//     if (at_end) {
//         ROS_INFO("TURTLE: Goal reached! Stopping navigation.");
//         futureMove.validAction = false;
//         return futureMove;
//     }

//     if (timeout_counter == 0) {
//         coordinate check_location = current_location;
        
//         switch (current_local_direction) {
//             case L_NORTH: {
//                 check_location.y = static_cast<uint8_t>(check_location.y - static_cast<uint8_t>(1));
//                 break;
//             }
//             case L_EAST: {
//                 check_location.x = static_cast<uint8_t>(check_location.x + static_cast<uint8_t>(1));
//                 break;
//             }
//             case L_SOUTH: {
//                 check_location.y = static_cast<uint8_t>(check_location.y + static_cast<uint8_t>(1));
//                 break;
//             }
//             case L_WEST: {
//                 check_location.x = static_cast<uint8_t>(check_location.x - static_cast<uint8_t>(1));
//                 break;
//             }
//             default: {
//                 ROS_ERROR("TURTLE: Invalid direction in check_location calculation");
//                 break;
//             }
//         }

//         uint8_t visitCount = getVisit(check_location);
//         bool canMoveForward = !bumpedIntoWall && (visitCount == 0);
//         ROS_INFO("TURTLE: Forward check - Can move: %s (Wall: %s, Visits: %d)", 
//                  canMoveForward ? "YES" : "NO", bumpedIntoWall ? "YES" : "NO", visitCount);

//         switch (current_state) {
//             case STATE_FORWARD: {
//                 ROS_INFO("TURTLE: In FORWARD state");
//                 if(canMoveForward) {
//                     ROS_INFO("TURTLE: Continuing forward");
//                     current_state = STATE_FORWARD;
//                 } else {
//                     ROS_INFO("TURTLE: Blocked - switching to CHECK_UNVISITED");
//                     current_state = STATE_UNVISITED;
//                     bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
//                     current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
//                     futureMove.action = RIGHT;
//                     directionsChecked++;
//                 }
//                 break;
//             }
//             case STATE_UNVISITED: {
//                 ROS_INFO("TURTLE: In CHECK_UNVISITED state");
//                 if(canMoveForward) {
//                     ROS_INFO("TURTLE: Found unvisited path - returning to FORWARD");
//                     current_state = STATE_FORWARD;
//                     directionsChecked = 0;
//                     for(int i = 0; i < 4; i++) {
//                         bumpedMap[i] = false;
//                     }
//                 } else if(directionsChecked < 4) {
//                     ROS_INFO("TURTLE: No path found - continuing to check directions");
//                     bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
//                     current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
//                     futureMove.action = RIGHT;
//                     directionsChecked++;
//                 } else {
//                     ROS_INFO("TURTLE: All directions checked - switching to CHECK_UNBUMPED");
//                     current_state = STATE_UNBUMPED;
//                     bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
//                     current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
//                     futureMove.action = RIGHT;
//                     directionsChecked = 0;
//                 }
//                 break;
//             }
//             case STATE_UNBUMPED: {
//                 ROS_INFO("TURTLE: In CHECK_UNBUMPED state");
//                 uint8_t minVisits = UINT8_MAX;
//                 LOCAL_DIRECTION minDirection = current_local_direction;
                
//                 for(int i = 0; i < 4; i++) {
//                     coordinate temp_location = current_location;
//                     switch (static_cast<LOCAL_DIRECTION>(i)) {
//                         case L_NORTH:
//                             temp_location.y = static_cast<uint8_t>(temp_location.y - static_cast<uint8_t>(1));
//                             break;
//                         case L_EAST:
//                             temp_location.x = static_cast<uint8_t>(temp_location.x + static_cast<uint8_t>(1));
//                             break;
//                         case L_SOUTH:
//                             temp_location.y = static_cast<uint8_t>(temp_location.y + static_cast<uint8_t>(1));
//                             break;
//                         case L_WEST:
//                             temp_location.x = static_cast<uint8_t>(temp_location.x - static_cast<uint8_t>(1));
//                             break;
//                         default:
//                             ROS_ERROR("TURTLE: Invalid direction in direction check");
//                             break;
//                     }
//                     uint8_t visit_count_map = getVisit(temp_location);
//                     ROS_INFO("TURTLE: Checking direction %d - Visits: %d, Wall: %s", 
//                              i, visit_count_map, bumpedMap[i] ? "YES" : "NO");
//                     if(!bumpedMap[i] && visit_count_map < minVisits) {
//                         minVisits = visit_count_map;
//                         minDirection = static_cast<LOCAL_DIRECTION>(i);
//                         ROS_INFO("TURTLE: New best direction: %d with %d visits", i, visit_count_map);
//                     }
//                 }
                
//                 if(current_local_direction != minDirection) {
//                     ROS_INFO("TURTLE: Rotating to direction %d", minDirection);
//                     current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
//                     futureMove.action = RIGHT;
//                 } else {
//                     ROS_INFO("TURTLE: Reached desired direction - returning to FORWARD");
//                     current_state = STATE_FORWARD;
//                     directionsChecked = 0;
//                     for(int i = 0; i < 4; i++) {
//                         bumpedMap[i] = false;
//                     }
//                 }
//                 break;
//             }
//             default: {
//                 ROS_ERROR("TURTLE: Invalid state in FSM");
//                 break;
//             }
//         }

//         if (current_state == STATE_FORWARD) {
//             ROS_INFO("TURTLE: Executing forward movement");
//             current_location = updateLocalTurtlePosition(current_location, current_local_direction);
//             uint8_t newVisitCount = static_cast<uint8_t>(getVisit(current_location) + static_cast<uint8_t>(1));
//             setVisit(current_location, newVisitCount);
//             futureMove.visitCount = getVisit(current_location);
//             futureMove.action = FORWARD;
//         }

//         timeout_counter = TIMEOUT;
//         futureMove.validAction = true;
        
//         ROS_INFO("TURTLE: Next move - Action: %s, Valid: %s, Visits: %d",
//                  futureMove.action == FORWARD ? "FORWARD" : 
//                  futureMove.action == LEFT ? "LEFT" : "RIGHT",
//                  futureMove.validAction ? "YES" : "NO",
//                  futureMove.visitCount);
                 
//         return futureMove;
//     }

//     timeout_counter--;
//     futureMove.validAction = false;
//     ROS_INFO("TURTLE: Timeout counter: %d", timeout_counter);
//     return futureMove;
// }
