/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/01/2024
 */
#include "student.h"

// Visit tracking grid
static uint8_t visit_grid[30][30] = {{0}};

// Helper function to get visit count for a position
uint8_t getVisitCount(int x, int y) {
    if (x >= 0 && x < 30 && y >= 0 && y < 30) {
        return visit_grid[x][y];
    }
    return UINT8_MAX;  // Return max for invalid positions
}

turtleMove studentTurtleStep(bool bumped_wall, bool at_goal) {
    static coordinate current_pos = {14, 14}; // Starting position
    static int facing_direction = 1;          // Start facing NORTH
    static bool first_move = true;
    
    turtleMove next_move;
    next_move.validAction = true;
    next_move.action = FORWARD;

    // Initialize first position
    if (first_move) {
        visit_grid[current_pos.x][current_pos.y]++;
        first_move = false;
        next_move.visitCount = visit_grid[current_pos.x][current_pos.y];
        return next_move;
    }

    // Stop at goal
    if (at_goal) {
        next_move.validAction = false;
        return next_move;
    }

    // Get visit counts for all adjacent cells
    uint8_t visit_counts[4];  // WEST, NORTH, EAST, SOUTH
    visit_counts[0] = getVisitCount(current_pos.x - 1, current_pos.y);     // WEST
    visit_counts[1] = getVisitCount(current_pos.x, current_pos.y - 1);     // NORTH
    visit_counts[2] = getVisitCount(current_pos.x + 1, current_pos.y);     // EAST
    visit_counts[3] = getVisitCount(current_pos.x, current_pos.y + 1);     // SOUTH

    // If current direction is blocked, mark it as maximum
    if (bumped_wall) {
        visit_counts[facing_direction] = UINT8_MAX;
    }

    // Find direction with minimum visits
    uint8_t min_visits = UINT8_MAX;
    int best_direction = -1;

    for (int dir = 0; dir < 4; dir++) {
        if (visit_counts[dir] < min_visits) {
            min_visits = visit_counts[dir];
            best_direction = dir;
        }
    }

    // If we can't find a valid direction
    if (best_direction == -1) {
        next_move.validAction = false;
        return next_move;
    }

    // If we're not facing the best direction, turn towards it
    if (best_direction != facing_direction) {
        // Calculate shortest turning direction
        int diff = (best_direction - facing_direction + 4) % 4;
        if (diff == 1) {
            next_move.action = RIGHT;
            facing_direction = (facing_direction + 1) % 4;
        } else {
            next_move.action = LEFT;
            facing_direction = (facing_direction + 3) % 4;
        }
    } 
    // If we're facing the best direction and no wall, move forward
    else if (!bumped_wall) {
        switch (facing_direction) {
            case 0: current_pos.x--; break; // WEST
            case 1: current_pos.y--; break; // NORTH
            case 2: current_pos.x++; break; // EAST
            case 3: current_pos.y++; break; // SOUTH
        }
        visit_grid[current_pos.x][current_pos.y]++;
        next_move.action = FORWARD;
        next_move.visitCount = visit_grid[current_pos.x][current_pos.y];
    }

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
