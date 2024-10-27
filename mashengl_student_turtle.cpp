/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

static int visit_map[MAZE_GRID_SIZE][MAZE_GRID_SIZE] = {0};
static Point current_pos = {0, 0};
static Orientation current_direction = NORTH;

// Find best direction to move
Orientation findBestDirection(bool is_bumped) {
    uint8_t min_visits = 255;
    Orientation best_dir = current_direction;
    QPointF check_pos(current_pos.x, current_pos.y);
    
    // Check all directions
    for (int dir = 0; dir < ORIENTATION_COUNT; dir++) {
        // Skip current direction if bumped
        if (dir == current_direction && is_bumped) continue;
        
        if (!detectObstacle(check_pos, static_cast<Orientation>(dir))) {
            QPointF next_pos = translatePos(check_pos, static_cast<Orientation>(dir));
            uint8_t visits = retrieveVisitCount(next_pos);
            
            // Prefer unvisited squares
            if (visits == 0) return static_cast<Orientation>(dir);
            
            if (visits < min_visits) {
                min_visits = visits;
                best_dir = static_cast<Orientation>(dir);
            }
        }
    }
    
    return best_dir;
}

// Calculate optimal turn direction
turtleMove getOptimalTurn(Orientation current, Orientation target) {
    int diff = (target - current + ORIENTATION_COUNT) % ORIENTATION_COUNT;
    if (diff == 0) return MOVE;
    return (diff == 1 || diff == 2) ? TURNRIGHT : TURNLEFT;
}

// Main solver function
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {
    if (goal) {
        *cur_state = AT_END;
        return STOP;
    }
    
    switch (*cur_state) {
        case CHECK_SURROUNDINGS: {
            Orientation target = findBestDirection(bumped);
            
            if (target != current_direction) {
                *cur_state = (getOptimalTurn(current_direction, target) == TURNRIGHT) ? 
                            TURN_RIGHT : TURN_LEFT;
                return getOptimalTurn(current_direction, target);
            }
            
            if (!bumped) {
                *cur_state = MOVE_FORWARD;
                return MOVE;
            }
            
            // If bumped and can't turn, try turning right
            *cur_state = TURN_RIGHT;
            return TURNRIGHT;
        }
        
        case MOVE_FORWARD: {
            if (!bumped) {
                // Update position tracking
                QPointF new_pos(current_pos.x, current_pos.y);
                new_pos = translatePos(new_pos, current_direction);
                current_pos.x = new_pos.x();
                current_pos.y = new_pos.y();
            }
            *cur_state = CHECK_SURROUNDINGS;
            return STOP;
        }
        
        case TURN_RIGHT:
        case TURN_LEFT: {
            current_direction = static_cast<Orientation>(
                (current_direction + (*cur_state == TURN_RIGHT ? 1 : 3)) % ORIENTATION_COUNT
            );
            *cur_state = CHECK_SURROUNDINGS;
            return STOP;
        }
        
        case AT_END:
        default:
            return STOP;
    }
}
