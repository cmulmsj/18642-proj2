/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/01/2024
 */

#ifdef testing
#include "student_mock.h"
#else
#include "student.h"
#include "ros/ros.h"
#endif

#ifndef testing
static FSM_STATES current_state = STATE_FORWARD;
static LOCAL_DIRECTION current_local_direction = L_NORTH;
static coordinate current_location = {14, 14};
static uint8_t visit_count_map[30][30] = {{0}};
static uint8_t timeout_counter = 5;
static uint8_t directions_checked = 0;
#endif

static uint8_t timeout_counter = 5;
static uint8_t directions_checked = 0;

/**
 * @brief Get number of visits for a cell
 * @param local_coord Grid position to check
 * @return Visit count for the specified cell
 */
uint8_t getVisit(coordinate local_coord) { 
#ifdef testing
    return get_test_state().visit_count_map[local_coord.x][local_coord.y];
#else
    uint8_t visit_count = visit_count_map[local_coord.x][local_coord.y];
    ROS_INFO("TURTLE: Getting visit count at (%d,%d): %d", 
             local_coord.x, local_coord.y, visit_count);
    return visit_count;
#endif
}

/**
 * @brief Update visit count for a cell
 * @param local_coord Grid position to update
 * @param setVal New visit count
 */
void setVisit(coordinate local_coord, uint8_t setVal) {
#ifdef testing
    TurtleTestState state = get_test_state();
    state.visit_count_map[local_coord.x][local_coord.y] = setVal;
    set_test_state(state);
#else
    ROS_INFO("TURTLE: Updating visit count at (%d,%d): %d -> %d", 
             local_coord.x, local_coord.y, visit_count_map[local_coord.x][local_coord.y], setVal);
    visit_count_map[local_coord.x][local_coord.y] = setVal;
#endif
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
    static bool first_run = true;

    if (first_run) {
#ifdef testing
        coordinate loc = {14, 14};
        setCurrentLocation(loc);
        setCurrentDirection(L_NORTH);
        setCurrentState(STATE_FORWARD);
#else
        current_location.x = 14;
        current_location.y = 14;
        first_run = false;
#endif
        ROS_INFO("TURTLE: Initialized starting position to (14,14)");
    }

    turtleMove futureMove;
    const uint8_t TIMEOUT = 5;

    ROS_INFO("\nTURTLE: === New Navigation Step ===");
    ROS_INFO("TURTLE: Wall detected: %s, At goal: %s", 
             bumpedIntoWall ? "YES" : "NO", at_end ? "YES" : "NO");
    ROS_INFO("TURTLE: Current State: %d, Directions Checked: %d", 
             getCurrentState(), directions_checked);

    if (at_end) {
        ROS_INFO("TURTLE: Goal reached! Stopping navigation.");
        futureMove.validAction = false;
        return futureMove;
    }

    if (timeout_counter == 0) {
        coordinate check_location;
#ifdef testing
        check_location = getCurrentLocation();
#else
        check_location = current_location;
#endif
        
        switch (getCurrentDirection()) {
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

        switch (getCurrentState()) {
            case STATE_FORWARD: {
                ROS_INFO("TURTLE: In FORWARD state");
                if(canMoveForward) {
                    ROS_INFO("TURTLE: Continuing forward");
#ifdef testing
                    setCurrentState(STATE_FORWARD);
#else
                    current_state = STATE_FORWARD;
#endif
                } else {
                    ROS_INFO("TURTLE: Blocked - switching to CHECK_UNVISITED");
#ifdef testing
                    setCurrentState(STATE_UNVISITED);
                    directions_checked = 1;
#else
                    current_state = STATE_UNVISITED;
                    directions_checked = 1;
#endif
                    futureMove.action = RIGHT;
                }
                break;
            }
            case STATE_UNVISITED: {
                ROS_INFO("TURTLE: In CHECK_UNVISITED state");
                if(canMoveForward) {
                    ROS_INFO("TURTLE: Found unvisited path - returning to FORWARD");
#ifdef testing
                    setCurrentState(STATE_FORWARD);
#else
                    current_state = STATE_FORWARD;
#endif
                } else if(directions_checked < 4) {
                    ROS_INFO("TURTLE: No path found - continuing to check directions");
#ifdef testing
                    setCurrentState(STATE_UNVISITED);
#else
                    current_state = STATE_UNVISITED;
#endif
                    directions_checked++;
                    futureMove.action = RIGHT;
                } else {
                    ROS_INFO("TURTLE: All directions checked - switching to CHECK_UNBUMPED");
#ifdef testing
                    setCurrentState(STATE_UNBUMPED);
#else
                    current_state = STATE_UNBUMPED;
#endif
                    directions_checked = 0;
                    futureMove.action = RIGHT;
                }
                break;
            }
            case STATE_UNBUMPED: {
                ROS_INFO("TURTLE: In CHECK_UNBUMPED state");
                uint8_t minVisits = UINT8_MAX;
                LOCAL_DIRECTION minDirection = getCurrentDirection();
                
                for(int i = 0; i < 4; i++) {
                    coordinate temp_location;
#ifdef testing
                    temp_location = getCurrentLocation();
#else
                    temp_location = current_location;
#endif
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
                    uint8_t visit_count = getVisit(temp_location);
                    ROS_INFO("TURTLE: Checking direction %d - Visits: %d, Wall: %s", 
                             i, visit_count, bumpedIntoWall ? "YES" : "NO");
                    if(!bumpedIntoWall && visit_count < minVisits) {
                        minVisits = visit_count;
                        minDirection = static_cast<LOCAL_DIRECTION>(i);
                        ROS_INFO("TURTLE: New best direction: %d with %d visits", i, visit_count);
                    }
                }
                
                if(getCurrentDirection() != minDirection) {
                    ROS_INFO("TURTLE: Rotating to direction %d", minDirection);
#ifdef testing
                    setCurrentDirection(static_cast<LOCAL_DIRECTION>((static_cast<int>(getCurrentDirection()) + 1) % 4));
#else
                    current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
#endif
                    futureMove.action = RIGHT;
                } else {
                    ROS_INFO("TURTLE: Reached desired direction - returning to FORWARD");
#ifdef testing
                    setCurrentState(STATE_FORWARD);
#else
                    current_state = STATE_FORWARD;
#endif
                }
                break;
            }
            default: {
                ROS_ERROR("TURTLE: Invalid state in FSM");
                break;
            }
        }

        if (getCurrentState() == STATE_FORWARD) {
            ROS_INFO("TURTLE: Executing forward movement");
#ifdef testing
            coordinate currentLoc = getCurrentLocation();
            LOCAL_DIRECTION currentDir = getCurrentDirection();
            setCurrentLocation(updateLocalTurtlePosition(currentLoc, currentDir));
#else
            current_location = updateLocalTurtlePosition(current_location, current_local_direction);
#endif
            uint8_t newVisitCount = static_cast<uint8_t>(getVisit(getCurrentLocation()) + static_cast<uint8_t>(1));
            setVisit(getCurrentLocation(), newVisitCount);
            futureMove.visitCount = getVisit(getCurrentLocation());
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
