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
#else
#include "student.h"
#include "ros/ros.h"
#endif

// Remove these function definitions from student_turtle.cpp since they'll be in mock_functions.cpp
#ifndef testing
static FSM_STATES current_state = STATE_FORWARD;
static LOCAL_DIRECTION current_local_direction = L_NORTH;
static coordinate current_location = {14, 14};
static uint8_t visit_count_map[30][30] = {{0}};

FSM_STATES getCurrentState() {
    return current_state;
}

void setCurrentState(FSM_STATES state) {
    current_state = state;
}

LOCAL_DIRECTION getCurrentDirection() {
    return current_local_direction;
}

void setCurrentDirection(LOCAL_DIRECTION dir) {
    current_local_direction = dir;
}

coordinate getCurrentLocation() {
    return current_location;
}

void setCurrentLocation(coordinate loc) {
    current_location = loc;
}

void resetVisitMap() {
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            visit_count_map[i][j] = 0;
        }
    }
}
#endif

/**
 * @brief Get number of visits for a cell
 * @param local_coord Grid position to check
 * @return Visit count for the specified cell
 */
uint8_t getVisit(coordinate local_coord) { 
    uint8_t visit_count = visit_count_map[local_coord.x][local_coord.y];
    ROS_INFO("TURTLE: Getting visit count at (%d,%d): %d", 
             local_coord.x, local_coord.y, visit_count);
    return visit_count;
}

/**
 * @brief Update visit count for a cell
 * @param local_coord Grid position to update
 * @param setVal New visit count
 */
void setVisit(coordinate local_coord, uint8_t setVal) {
    ROS_INFO("TURTLE: Updating visit count at (%d,%d): %d -> %d", 
             local_coord.x, local_coord.y, visit_count_map[local_coord.x][local_coord.y], setVal);
    visit_count_map[local_coord.x][local_coord.y] = setVal;
}

/**
 * @brief Calculate next grid position based on current position and direction
 * @param current_location Current position
 * @param local_orientation Current direction
 * @return New position after movement
 */
coordinate updateLocalTurtlePosition(coordinate current_location, LOCAL_DIRECTION local_orientation) {
    coordinate new_location = current_location;
    ROS_INFO("TURTLE: Current position (%d,%d), Direction: %d", 
             current_location.x, current_location.y, local_orientation);

    switch (local_orientation) {
        case L_NORTH: {
            new_location.y = static_cast<uint8_t>(new_location.y - static_cast<uint8_t>(1));
            ROS_INFO("TURTLE: Moving NORTH");
            break;
        }
        case L_EAST: {
            new_location.x = static_cast<uint8_t>(new_location.x + static_cast<uint8_t>(1));
            ROS_INFO("TURTLE: Moving EAST");
            break;
        }
        case L_SOUTH: {
            new_location.y = static_cast<uint8_t>(new_location.y + static_cast<uint8_t>(1));
            ROS_INFO("TURTLE: Moving SOUTH");
            break;
        }
        case L_WEST: {
            new_location.x = static_cast<uint8_t>(new_location.x - static_cast<uint8_t>(1));
            ROS_INFO("TURTLE: Moving WEST");
            break;
        }
        default: {
            ROS_ERROR("TURTLE: Invalid direction in updateLocalTurtlePosition: %d", local_orientation);
            break;
        }
    }
    
    ROS_INFO("TURTLE: New position: (%d,%d)", new_location.x, new_location.y);
    return new_location;
}

/**
 * @brief Main navigation algorithm implementation
 * 
 * Implements a finite state machine with three states:
 * 1. STATE_FORWARD: Moving through unvisited cells
 * 2. STATE_UNVISITED: Looking for unvisited paths when stuck
 * 3. STATE_UNBUMPED: Finding optimal path when surrounded by visited cells
 * 
 * @param bumpedIntoWall True if wall detected ahead
 * @param at_end True if goal reached
 * @return Next movement command
 */
turtleMove studentTurtleStep(bool bumpedIntoWall, bool at_end) {
    // Initialize static variables
    static coordinate current_location;
    static LOCAL_DIRECTION current_local_direction = L_NORTH;
    static uint8_t directionsChecked = 0;
    static FSM_STATES current_state = STATE_FORWARD;
    static bool bumpedMap[4] = {false};
    static bool first_run = true;

    if (first_run) {
        current_location.x = 14;
        current_location.y = 14;
        first_run = false;
        ROS_INFO("TURTLE: Initialized starting position to (14,14)");
    }

    turtleMove futureMove;
    const uint8_t TIMEOUT = 5;
    static uint8_t timeout_counter;

    ROS_INFO("\nTURTLE: === New Navigation Step ===");
    ROS_INFO("TURTLE: Wall detected: %s, At goal: %s", 
             bumpedIntoWall ? "YES" : "NO", at_end ? "YES" : "NO");
    ROS_INFO("TURTLE: Current State: %d, Directions Checked: %d", 
             current_state, directionsChecked);

    if (at_end) {
        ROS_INFO("TURTLE: Goal reached! Stopping navigation.");
        futureMove.validAction = false;
        return futureMove;
    }

    if (timeout_counter == 0) {
        coordinate check_location = current_location;
        
        switch (current_local_direction) {
            case L_NORTH: {
                check_location.y = static_cast<uint8_t>(check_location.y - static_cast<uint8_t>(1));
                break;
            }
            case L_EAST: {
                check_location.x = static_cast<uint8_t>(check_location.x + static_cast<uint8_t>(1));
                break;
            }
            case L_SOUTH: {
                check_location.y = static_cast<uint8_t>(check_location.y + static_cast<uint8_t>(1));
                break;
            }
            case L_WEST: {
                check_location.x = static_cast<uint8_t>(check_location.x - static_cast<uint8_t>(1));
                break;
            }
            default: {
                ROS_ERROR("TURTLE: Invalid direction in check_location calculation");
                break;
            }
        }

        uint8_t visitCount = getVisit(check_location);
        bool canMoveForward = !bumpedIntoWall && (visitCount == 0);
        ROS_INFO("TURTLE: Forward check - Can move: %s (Wall: %s, Visits: %d)", 
                 canMoveForward ? "YES" : "NO", bumpedIntoWall ? "YES" : "NO", visitCount);

        switch (current_state) {
            case STATE_FORWARD: {
                ROS_INFO("TURTLE: In FORWARD state");
                if(canMoveForward) {
                    ROS_INFO("TURTLE: Continuing forward");
                    current_state = STATE_FORWARD;
                } else {
                    ROS_INFO("TURTLE: Blocked - switching to CHECK_UNVISITED");
                    current_state = STATE_UNVISITED;
                    bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
                    current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
                    futureMove.action = RIGHT;
                    directionsChecked++;
                }
                break;
            }
            case STATE_UNVISITED: {
                ROS_INFO("TURTLE: In CHECK_UNVISITED state");
                if(canMoveForward) {
                    ROS_INFO("TURTLE: Found unvisited path - returning to FORWARD");
                    current_state = STATE_FORWARD;
                    directionsChecked = 0;
                    for(int i = 0; i < 4; i++) {
                        bumpedMap[i] = false;
                    }
                } else if(directionsChecked < 4) {
                    ROS_INFO("TURTLE: No path found - continuing to check directions");
                    bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
                    current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
                    futureMove.action = RIGHT;
                    directionsChecked++;
                } else {
                    ROS_INFO("TURTLE: All directions checked - switching to CHECK_UNBUMPED");
                    current_state = STATE_UNBUMPED;
                    bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
                    current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
                    futureMove.action = RIGHT;
                    directionsChecked = 0;
                }
                break;
            }
            case STATE_UNBUMPED: {
                ROS_INFO("TURTLE: In CHECK_UNBUMPED state");
                uint8_t minVisits = UINT8_MAX;
                LOCAL_DIRECTION minDirection = current_local_direction;
                
                for(int i = 0; i < 4; i++) {
                    coordinate temp_location = current_location;
                    switch (static_cast<LOCAL_DIRECTION>(i)) {
                        case L_NORTH:
                            temp_location.y = static_cast<uint8_t>(temp_location.y - static_cast<uint8_t>(1));
                            break;
                        case L_EAST:
                            temp_location.x = static_cast<uint8_t>(temp_location.x + static_cast<uint8_t>(1));
                            break;
                        case L_SOUTH:
                            temp_location.y = static_cast<uint8_t>(temp_location.y + static_cast<uint8_t>(1));
                            break;
                        case L_WEST:
                            temp_location.x = static_cast<uint8_t>(temp_location.x - static_cast<uint8_t>(1));
                            break;
                        default:
                            ROS_ERROR("TURTLE: Invalid direction in direction check");
                            break;
                    }
                    uint8_t visit_count_map = getVisit(temp_location);
                    ROS_INFO("TURTLE: Checking direction %d - Visits: %d, Wall: %s", 
                             i, visit_count_map, bumpedMap[i] ? "YES" : "NO");
                    if(!bumpedMap[i] && visit_count_map < minVisits) {
                        minVisits = visit_count_map;
                        minDirection = static_cast<LOCAL_DIRECTION>(i);
                        ROS_INFO("TURTLE: New best direction: %d with %d visits", i, visit_count_map);
                    }
                }
                
                if(current_local_direction != minDirection) {
                    ROS_INFO("TURTLE: Rotating to direction %d", minDirection);
                    current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
                    futureMove.action = RIGHT;
                } else {
                    ROS_INFO("TURTLE: Reached desired direction - returning to FORWARD");
                    current_state = STATE_FORWARD;
                    directionsChecked = 0;
                    for(int i = 0; i < 4; i++) {
                        bumpedMap[i] = false;
                    }
                }
                break;
            }
            default: {
                ROS_ERROR("TURTLE: Invalid state in FSM");
                break;
            }
        }

        if (current_state == STATE_FORWARD) {
            ROS_INFO("TURTLE: Executing forward movement");
            current_location = updateLocalTurtlePosition(current_location, current_local_direction);
            uint8_t newVisitCount = static_cast<uint8_t>(getVisit(current_location) + static_cast<uint8_t>(1));
            setVisit(current_location, newVisitCount);
            futureMove.visitCount = getVisit(current_location);
            futureMove.action = FORWARD;
        }

        timeout_counter = TIMEOUT;
        futureMove.validAction = true;
        
        ROS_INFO("TURTLE: Next move - Action: %s, Valid: %s, Visits: %d",
                 futureMove.action == FORWARD ? "FORWARD" : 
                 futureMove.action == LEFT ? "LEFT" : "RIGHT",
                 futureMove.validAction ? "YES" : "NO",
                 futureMove.visitCount);
                 
        return futureMove;
    }

    timeout_counter--;
    futureMove.validAction = false;
    ROS_INFO("TURTLE: Timeout counter: %d", timeout_counter);
    return futureMove;
}
