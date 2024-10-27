/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// Visit tracking grid
static uint8_t grid_visits[MAZE_SIZE][MAZE_SIZE] = {{0}};

// FSM States (matching Yuma's implementation)
enum FSM_STATE { 
    FORWARD_STATE = 0,
    CHECK_UNVISITED_STATE = 1,
    CHECK_UNBUMPED_STATE = 2
};

// Local direction enumeration
enum LOCAL_DIR { L_WEST = 0, L_NORTH = 1, L_EAST = 2, L_SOUTH = 3 };

// Current position tracking
static coordinate current_pos = {START_POS, START_POS};
static LOCAL_DIR current_dir = L_NORTH;
static uint8_t directions_checked = 0;
static bool wall_map[MAX_DIRECTIONS] = {false};

// Get visit count for position
uint8_t getVisits(coordinate pos) { 
    return grid_visits[pos.x][pos.y]; 
}

// Set visit count for position
void setVisits(coordinate pos, uint8_t val) {
    grid_visits[pos.x][pos.y] = val;
}

// Get next cell coordinates
coordinate getNextCell(coordinate pos, LOCAL_DIR dir) {
    coordinate next = pos;
    switch (dir) {
        case L_NORTH: next.y--; break;
        case L_EAST:  next.x++; break;
        case L_SOUTH: next.y++; break;
        case L_WEST:  next.x--; break;
    }
    ROS_INFO("[NEXT] From (%d,%d) dir %d -> (%d,%d)", 
             pos.x, pos.y, dir, next.x, next.y);
    return next;
}

// Check if forward move is ideal
bool isIdealMove(bool wall_hit, coordinate pos, LOCAL_DIR dir) {
    if (wall_hit) return false;
    coordinate next = getNextCell(pos, dir);
    return (getVisits(next) == 0);
}

// Find direction with least visits
LOCAL_DIR findBestMove(coordinate pos, bool* walls) {
    uint8_t min_visits = 255;
    LOCAL_DIR best_dir = current_dir;
    
    for(int i = 0; i < MAX_DIRECTIONS; i++) {
        if(walls[i]) continue;
        coordinate next = getNextCell(pos, static_cast<LOCAL_DIR>(i));
        uint8_t visits = getVisits(next);
        ROS_INFO("[SCAN] Dir %d has %d visits", i, visits);
        
        if(visits < min_visits) {
            min_visits = visits;
            best_dir = static_cast<LOCAL_DIR>(i);
            ROS_INFO("[SCAN] New best dir %d", i);
        }
    }
    return best_dir;
}

// Main solver function
turtleMove studentTurtleStep(bool wall_hit, bool at_end) {
    static FSM_STATE state = FORWARD_STATE;
    turtleMove next_move = {NONE, true, 0};
    
    ROS_INFO("[STATE] Current: %d at (%d,%d) facing %d", 
             state, current_pos.x, current_pos.y, current_dir);

    if (at_end) {
        next_move.validAction = false;
        return next_move;
    }

    switch(state) {
        case FORWARD_STATE: {
            if(!isIdealMove(wall_hit, current_pos, current_dir)) {
                ROS_INFO("[FWD] Need to check new directions");
                state = CHECK_UNVISITED_STATE;
                wall_map[current_dir] = wall_hit;
                current_dir = static_cast<LOCAL_DIR>((current_dir + 1) % 4);
                next_move.action = RIGHT;
                directions_checked = 1;
            } else {
                ROS_INFO("[FWD] Moving forward");
                next_move.action = FORWARD;
                current_pos = getNextCell(current_pos, current_dir);
                setVisits(current_pos, getVisits(current_pos) + 1);
                next_move.visitCount = getVisits(current_pos);
            }
            break;
        }

        case CHECK_UNVISITED_STATE: {
            if(!isIdealMove(wall_hit, current_pos, current_dir)) {
                if(directions_checked < 4) {
                    ROS_INFO("[CHECK] Continue checking, dir %d", directions_checked);
                    wall_map[current_dir] = wall_hit;
                    current_dir = static_cast<LOCAL_DIR>((current_dir + 1) % 4);
                    next_move.action = RIGHT;
                    directions_checked++;
                } else {
                    ROS_INFO("[CHECK] All directions checked");
                    state = CHECK_UNBUMPED_STATE;
                    wall_map[current_dir] = wall_hit;
                    current_dir = static_cast<LOCAL_DIR>((current_dir + 1) % 4);
                    next_move.action = RIGHT;
                }
            } else {
                ROS_INFO("[CHECK] Found good direction");
                state = FORWARD_STATE;
                memset(wall_map, 0, MAX_DIRECTIONS);
                next_move.action = FORWARD;
                current_pos = getNextCell(current_pos, current_dir);
                setVisits(current_pos, getVisits(current_pos) + 1);
                next_move.visitCount = getVisits(current_pos);
                directions_checked = 0;
            }
            break;
        }

        case CHECK_UNBUMPED_STATE: {
            LOCAL_DIR best_dir = findBestMove(current_pos, wall_map);
            if(current_dir != best_dir) {
                ROS_INFO("[UNBUMP] Turning to dir %d", best_dir);
                current_dir = static_cast<LOCAL_DIR>((current_dir + 1) % 4);
                next_move.action = RIGHT;
            } else {
                ROS_INFO("[UNBUMP] Ready to move forward");
                state = FORWARD_STATE;
                memset(wall_map, 0, MAX_DIRECTIONS);
                directions_checked = 0;
            }
            break;
        }
    }

    ROS_INFO("[MOVE] Action: %d, Visits: %d\n", 
             next_move.action, next_move.visitCount);
    return next_move;
}
