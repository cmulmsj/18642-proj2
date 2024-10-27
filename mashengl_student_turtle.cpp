/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
 */

#include "student.h"

// Solver state machine states
enum SolverState {
    SCANNING,    // Scanning surroundings
    TURNING,     // Executing a turn
    MOVING,      // Moving forward
    BACKTRACK    // Backtracking from dead end
};

// State tracking
static struct {
    SolverState current_state = SCANNING;
    int scan_direction = 0;  // 0-3 for directions being scanned
    int target_direction = 0;
    Point current_pos = {0, 0};
    int current_orientation = 1; // Start facing North
} solver_state;

// Get coordinates of adjacent square in specified direction
Point get_adjacent_coords(Point current, int direction) {
    Point next = current;
    switch(direction) {
        case 0: next.x--; break; // West
        case 1: next.y--; break; // North
        case 2: next.x++; break; // East
        case 3: next.y++; break; // South
    }
    return next;
}

// Calculate optimal number of turns needed
int get_turn_count(int current_dir, int target_dir) {
    int diff = (target_dir - current_dir + 4) % 4;
    return (diff <= 2) ? diff : 4 - diff;
}

// Find direction with lowest visit count
int find_best_direction(Point current_pos, int current_orientation, bool is_bumped) {
    int min_visits = 999;
    int best_dir = current_orientation;
    
    // Check all directions
    for (int dir = 0; dir < 4; dir++) {
        // Skip current direction if bumped
        if (dir == current_orientation && is_bumped) continue;
        
        Point next = get_adjacent_coords(current_pos, dir);
        int visits = getVisitNumber(next);
        
        // Prefer unvisited squares
        if (visits == 0) return dir;
        
        // Otherwise take least visited
        if (visits < min_visits) {
            min_visits = visits;
            best_dir = dir;
        }
    }
    
    return best_dir;
}

// Main solver function
turtleMove studentTurtleStep(bool bumped, bool at_end) {
    if (at_end) return STOP;
    
    turtleMove next_move = STOP;
    
    switch(solver_state.current_state) {
        case SCANNING: {
            // Find best direction to move
            int best_dir = find_best_direction(solver_state.current_pos, 
                                             solver_state.current_orientation, 
                                             bumped);
            
            // If we need to turn
            if (best_dir != solver_state.current_orientation) {
                solver_state.target_direction = best_dir;
                solver_state.current_state = TURNING;
                // Decide turn direction (prefer right for single turn)
                next_move = (get_turn_count(solver_state.current_orientation, best_dir) == 1) ? 
                           TURNRIGHT : TURNLEFT;
            } else if (!bumped) {
                // Can move forward
                solver_state.current_state = MOVING;
                next_move = MOVE;
            } else {
                // Bumped and can't turn to better square, backtrack
                solver_state.current_state = BACKTRACK;
                next_move = TURNRIGHT;
            }
            break;
        }
        
        case TURNING: {
            if (solver_state.current_orientation == solver_state.target_direction) {
                solver_state.current_state = MOVING;
                next_move = MOVE;
            } else {
                next_move = (get_turn_count(solver_state.current_orientation, 
                           solver_state.target_direction) == 1) ? 
                           TURNRIGHT : TURNLEFT;
            }
            break;
        }
        
        case MOVING: {
            solver_state.current_state = SCANNING;
            if (!bumped) {
                // Update position tracking
                solver_state.current_pos = get_adjacent_coords(solver_state.current_pos, 
                                                            solver_state.current_orientation);
            }
            next_move = STOP;
            break;
        }
        
        case BACKTRACK: {
            // Complete the turn around
            solver_state.current_state = SCANNING;
            next_move = TURNRIGHT;
            break;
        }
    }
    
    // Update orientation tracking
    if (next_move == TURNRIGHT) {
        solver_state.current_orientation = (solver_state.current_orientation + 1) % 4;
    } else if (next_move == TURNLEFT) {
        solver_state.current_orientation = (solver_state.current_orientation + 3) % 4;
    }
    
    return next_move;
}

