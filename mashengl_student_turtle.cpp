/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// Visit tracking grid - initialized with zeros
static uint8_t grid_visits[MAZE_SIZE][MAZE_SIZE] = {{0}};

// Current position tracking - starts at maze center
static struct {
    coordinate pos{START_POS, START_POS};
    uint8_t facing = 1;  // Start facing North
    uint8_t next_facing = 1;
    uint8_t scan_count = 0;
    bool walls[MAX_DIRECTIONS] = {false};
} turtle;

// FSM control states
enum MazeState { 
    FORWARD_SCAN = 0,    // Initial scanning
    ROTATE_SCAN = 1,     // Rotation during scanning
    FIND_DIRECTION = 2   // Find best direction
};

// Current FSM state
static MazeState curr_state = FORWARD_SCAN;

// Get visit count for a position
uint8_t getVisits(coordinate pos) {
    return grid_visits[pos.x][pos.y];
}

// Update visit count for a position
void setVisits(coordinate pos, uint8_t val) {
    grid_visits[pos.x][pos.y] = val;
    ROS_INFO("[VISITS] Set (%d,%d) to %d visits", pos.x, pos.y, val);
}

// Get coordinates after a move in given direction
coordinate getNextPos(coordinate current, uint8_t dir) {
    coordinate next = current;
    switch (dir) {
        case 0: next.x--; break;        // West
        case 1: next.y--; break;        // North
        case 2: next.x++; break;        // East
        case 3: next.y++; break;        // South
    }
    ROS_INFO("[NEXT] From (%d,%d) dir %d -> (%d,%d)", 
             current.x, current.y, dir, next.x, next.y);
    return next;
}

// Find least visited accessible direction
uint8_t findBestDir(coordinate pos, bool* walls) {
    uint8_t min_visits = 255;
    uint8_t best_dir = turtle.facing;
    
    ROS_INFO("[SCAN] Current pos (%d,%d) facing %d", pos.x, pos.y, turtle.facing);
    
    for(uint8_t dir = 0; dir < MAX_DIRECTIONS; dir++) {
        if(walls[dir]) {
            ROS_INFO("[SCAN] Dir %d blocked by wall", dir);
            continue;
        }
        
        coordinate next = getNextPos(pos, dir);
        uint8_t visits = getVisits(next);
        
        ROS_INFO("[SCAN] Dir %d has %d visits", dir, visits);
        
        if(visits == 0) {
            ROS_INFO("[SCAN] Found unvisited direction %d!", dir);
            return dir;
        }
        
        if(visits < min_visits) {
            min_visits = visits;
            best_dir = dir;
            ROS_INFO("[SCAN] New best dir %d with %d visits", dir, visits);
        }
    }
    
    return best_dir;
}

// Main solver function
turtleMove studentTurtleStep(bool has_wall, bool reached_end) {
    turtleMove next_move = {NONE, true, 0};
    
    ROS_INFO("\n[STATE] Current: %d, Pos: (%d,%d), Facing: %d", 
             curr_state, turtle.pos.x, turtle.pos.y, turtle.facing);
    ROS_INFO("[STATUS] Wall: %d, End: %d", has_wall, reached_end);

    // Stop if we reached the end
    if(reached_end) {
        ROS_INFO("[END] Reached maze end!");
        next_move.validAction = false;
        return next_move;
    }

    // Main FSM logic
    switch(curr_state) {
        case FORWARD_SCAN: {
            coordinate next = getNextPos(turtle.pos, turtle.facing);
            bool is_new = (getVisits(next) == 0);
            
            if(!has_wall && is_new) {
                ROS_INFO("[FWD] Moving forward to unvisited cell");
                next_move.action = FORWARD;
                turtle.pos = next;
                setVisits(turtle.pos, getVisits(turtle.pos) + 1);
                next_move.visitCount = getVisits(turtle.pos);
            } else {
                ROS_INFO("[FWD] Need to scan, wall=%d, new=%d", has_wall, is_new);
                curr_state = ROTATE_SCAN;
                turtle.walls[turtle.facing] = has_wall;
                turtle.next_facing = (turtle.facing + 1) % MAX_DIRECTIONS;
                next_move.action = RIGHT;
                turtle.scan_count = 1;
            }
            break;
        }

        case ROTATE_SCAN: {
            if(has_wall) turtle.walls[turtle.facing] = true;
            
            if(turtle.scan_count < MAX_DIRECTIONS) {
                ROS_INFO("[ROTATE] Continuing scan, checked %d dirs", turtle.scan_count);
                turtle.next_facing = (turtle.facing + 1) % MAX_DIRECTIONS;
                next_move.action = RIGHT;
                turtle.scan_count++;
            } else {
                ROS_INFO("[ROTATE] Scan complete, finding best direction");
                curr_state = FIND_DIRECTION;
                turtle.next_facing = findBestDir(turtle.pos, turtle.walls);
                next_move.action = RIGHT;
            }
            break;
        }

        case FIND_DIRECTION: {
            if(turtle.facing == turtle.next_facing) {
                ROS_INFO("[FIND] Aligned to best direction %d, moving forward", turtle.facing);
                curr_state = FORWARD_SCAN;
                next_move.action = FORWARD;
                turtle.pos = getNextPos(turtle.pos, turtle.facing);
                setVisits(turtle.pos, getVisits(turtle.pos) + 1);
                next_move.visitCount = getVisits(turtle.pos);
                memset(turtle.walls, 0, MAX_DIRECTIONS);
                turtle.scan_count = 0;
            } else {
                ROS_INFO("[FIND] Turning to align with direction %d", turtle.next_facing);
                next_move.action = RIGHT;
            }
            break;
        }
    }
    
    // Update orientation if turning
    if(next_move.action == RIGHT) {
        turtle.facing = turtle.next_facing;
    }
    
    ROS_INFO("[MOVE] Action: %d, Valid: %d, Visits: %d\n", 
             next_move.action, next_move.validAction, next_move.visitCount);
    
    return next_move;
}
