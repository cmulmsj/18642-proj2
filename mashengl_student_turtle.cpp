/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// Track turtle's state between calls
static struct TurtleData {
    Orientation facing = NORTH;         // Current orientation
    Orientation target_dir = NORTH;     // Direction we want to face
} turtle_data;

// Find direction with least visits
Orientation findBestDirection(QPointF current_pos, bool is_bumped) {
    uint8_t min_visits = 255;
    Orientation best_dir = turtle_data.facing;
    
    // Check all directions
    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        // Skip current direction if bumped
        if (i == turtle_data.facing && is_bumped) continue;
        
        // Get position in this direction
        QPointF test_pos = current_pos;
        switch (i) {
            case WEST:  test_pos.setX(current_pos.x() - 1); break;
            case EAST:  test_pos.setX(current_pos.x() + 1); break;
            case NORTH: test_pos.setY(current_pos.y() - 1); break;
            case SOUTH: test_pos.setY(current_pos.y() + 1); break;
        }
        
        // Skip if there's a wall
        if (detectObstacle(current_pos, static_cast<Orientation>(i))) {
            continue;
        }
        
        uint8_t visits = retrieveVisitCount(test_pos);
        
        // Prefer unvisited cells
        if (visits == 0) {
            return static_cast<Orientation>(i);
        }
        
        if (visits < min_visits) {
            min_visits = visits;
            best_dir = static_cast<Orientation>(i);
        }
    }
    
    return best_dir;
}

// Get turn direction (LEFT or RIGHT) to reach target orientation
turtleMove getTurnDirection(Orientation current, Orientation target) {
    int diff = (target - current + ORIENTATION_COUNT) % ORIENTATION_COUNT;
    return (diff == 1) ? TURNRIGHT : TURNLEFT;
}

// Main solver function
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {    
    if (goal) {
        *cur_state = AT_END;
        return STOP;
    }
    
    switch (*cur_state) {
        case EXPLORE: {
            // If bumped or at start, find a new direction
            if (bumped || turtle_data.facing == turtle_data.target_dir) {
                turtle_data.target_dir = findBestDirection(QPointF(0, 0), bumped);
                
                // If need to turn
                if (turtle_data.target_dir != turtle_data.facing) {
                    *cur_state = TURNING;
                    return getTurnDirection(turtle_data.facing, turtle_data.target_dir);
                }
            }
            
            // Move forward if no turn needed
            return MOVE;
        }
        
        case TURNING: {
            // Update our orientation after turning
            if (turtle_data.target_dir != turtle_data.facing) {
                turtle_data.facing = static_cast<Orientation>(
                    (turtle_data.facing + 
                     (getTurnDirection(turtle_data.facing, turtle_data.target_dir) == TURNRIGHT ? 1 : 3)
                    ) % ORIENTATION_COUNT
                );
                return STOP;
            }
            
            // Turn complete, start moving
            *cur_state = EXPLORE;
            return MOVE;
        }
        
        case BACKTRACK: {
            // If all else fails, turn right and try again
            turtle_data.facing = static_cast<Orientation>((turtle_data.facing + 1) % ORIENTATION_COUNT);
            turtle_data.target_dir = turtle_data.facing;
            *cur_state = EXPLORE;
            return TURNRIGHT;
        }
        
        case AT_END:
        default:
            return STOP;
    }
}
