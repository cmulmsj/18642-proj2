/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// Visit tracking grid with center at START_POS
static uint8_t visits[30][30] = {{0}};

// Current position tracking
static coordinate curr_pos = {14, 14};
static LOCAL_DIR curr_dir = L_NORTH;
static uint8_t dirs_checked = 0;
static bool wall_status[4] = {false};

// FSM States
enum FSM_STATE { 
    FORWARD_STATE = 0,
    CHECK_UNVISITED_STATE = 1,
    CHECK_UNBUMPED_STATE = 2
};

// Local directions
enum LOCAL_DIR { 
    L_WEST = 0, 
    L_NORTH = 1, 
    L_EAST = 2, 
    L_SOUTH = 3 
};

// Get visit count for a position
uint8_t getVisits(coordinate pos) { 
    ROS_INFO("[VISIT] Get (%d,%d): %d", pos.x, pos.y, visits[pos.x][pos.y]);
    return visits[pos.x][pos.y]; 
}

// Set visit count for a position
void setVisits(coordinate pos, uint8_t val) {
    ROS_INFO("[VISIT] Set (%d,%d) to %d", pos.x, pos.y, val);
    visits[pos.x][pos.y] = val;
}

// Get next position based on direction
coordinate getNextPos(coordinate pos, LOCAL_DIR dir) {
    coordinate next = pos;
    switch (dir) {
        case L_NORTH: next.y--; break;
        case L_EAST:  next.x++; break;
        case L_SOUTH: next.y++; break;
        case L_WEST:  next.x--; break;
    }
    return next;
}

// Check if move forward is ideal (unvisited and no wall)
bool isIdealMove(bool wall_ahead, coordinate pos, LOCAL_DIR dir) {
    coordinate next = getNextPos(pos, dir);
    uint8_t visit_count = getVisits(next);
    ROS_INFO("[CHECK] Forward from (%d,%d) dir %d: wall=%d visits=%d", 
             pos.x, pos.y, dir, wall_ahead, visit_count);
    return (!wall_ahead && visit_count == 0);
}

// Find least visited accessible direction
LOCAL_DIR getLeastVisited(coordinate pos, bool* walls) {
    uint8_t min_visits = 255;
    LOCAL_DIR best_dir = L_NORTH;
    
    for(int i = 0; i < 4; i++) {
        coordinate next = getNextPos(pos, static_cast<LOCAL_DIR>(i));
        uint8_t visits = getVisits(next);
        ROS_INFO("[SCAN] Dir %d - wall=%d visits=%d", i, walls[i], visits);
        
        if(!walls[i] && visits < min_visits) {
            min_visits = visits;
            best_dir = static_cast<LOCAL_DIR>(i);
            ROS_INFO("[SCAN] New best direction: %d", i);
        }
    }
    return best_dir;
}

// Main solver function
turtleMove studentTurtleStep(bool bumped, bool at_end) {
    static FSM_STATE state = FORWARD_STATE;
    turtleMove next_move = {NONE, true, 0};

    ROS_INFO("\n[STATE] Current: %d, Pos: (%d,%d), Dir: %d", 
             state, curr_pos.x, curr_pos.y, curr_dir);
    
    // Handle goal reached
    if (at_end) {
        next_move.validAction = false;
        return next_move;
    }

    // Main state machine
    switch(state) {
        case FORWARD_STATE: {
            if(isIdealMove(bumped, curr_pos, curr_dir)) {
                // Continue forward
                next_move.action = FORWARD;
                curr_pos = getNextPos(curr_pos, curr_dir);
                setVisits(curr_pos, getVisits(curr_pos) + 1);
                next_move.visitCount = getVisits(curr_pos);
            } else {
                // Need to check other directions
                state = CHECK_UNVISITED_STATE;
                wall_status[curr_dir] = bumped;
                curr_dir = static_cast<LOCAL_DIR>((curr_dir + 1) % 4);
                next_move.action = RIGHT;
                dirs_checked = 1;
            }
            break;
        }

        case CHECK_UNVISITED_STATE: {
            if(dirs_checked < 4) {
                wall_status[curr_dir] = bumped;
                curr_dir = static_cast<LOCAL_DIR>((curr_dir + 1) % 4);
                next_move.action = RIGHT;
                dirs_checked++;
            } else {
                state = CHECK_UNBUMPED_STATE;
                next_move.action = RIGHT;
                dirs_checked = 0;
            }
            break;
        }

        case CHECK_UNBUMPED_STATE: {
            LOCAL_DIR best_dir = getLeastVisited(curr_pos, wall_status);
            if(curr_dir != best_dir) {
                curr_dir = static_cast<LOCAL_DIR>((curr_dir + 1) % 4);
                next_move.action = RIGHT;
            } else {
                state = FORWARD_STATE;
                memset(wall_status, 0, sizeof(wall_status));
                next_move.action = FORWARD;
                curr_pos = getNextPos(curr_pos, curr_dir);
                setVisits(curr_pos, getVisits(curr_pos) + 1);
                next_move.visitCount = getVisits(curr_pos);
            }
            break;
        }
    }

    ROS_INFO("[MOVE] Action: %d, Valid: %d, Visits: %d\n", 
             next_move.action, next_move.validAction, next_move.visitCount);
    return next_move;
}
