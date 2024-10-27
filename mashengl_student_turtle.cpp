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
    Point pos = {0, 0};                // Current position
    Orientation facing = NORTH;         // Current orientation
    Orientation target_dir = NORTH;     // Direction we want to face
    bool completing_turn = false;       // True if in middle of turn sequence
} turtle_data;

// Find direction with least visits
Orientation findBestDirection(QPointF current_pos, Orientation current_dir, bool is_bumped) {
    int min_visits = 999;
    Orientation best_dir = current_dir;
    
    // Check all directions
    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        if (i == current_dir && is_bumped) continue;
        
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
        
        int visits = retrieveVisitCount(test_pos);
        
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

// Main solver function
turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {    
    if (goal) {
        *cur_state = AT_END;
        return STOP;
    }
    
    QPointF current_pos(turtle_data.pos.x, turtle_data.pos.y);
    
    switch (*cur_state) {
        case EXPLORE: {
            if (bumped) {
                // Hit wall, need to turn
                turtle_data.target_dir = findBestDirection(current_pos, turtle_data.facing, true);
                if (turtle_data.target_dir != turtle_data.facing) {
                    *cur_state = TURNING;
                    turtle_data.completing_turn = true;
                    return ((turtle_data.target_dir - turtle_data.facing + ORIENTATION_COUNT) % ORIENTATION_COUNT == 1) ? 
                           TURNRIGHT : TURNLEFT;
                }
                return STOP;
            }
            
            // Check for better direction
            turtle_data.target_dir = findBestDirection(current_pos, turtle_data.facing, false);
            if (turtle_data.target_dir != turtle_data.facing) {
                *cur_state = TURNING;
                turtle_data.completing_turn = true;
                return ((turtle_data.target_dir - turtle_data.facing + ORIENTATION_COUNT) % ORIENTATION_COUNT == 1) ? 
                       TURNRIGHT : TURNLEFT;
            }
            
            // Move forward if no better direction
            return MOVE;
        }
        
        case TURNING: {
            if (turtle_data.completing_turn) {
                turtle_data.completing_turn = false;
                turtle_data.facing = turtle_data.target_dir;
                *cur_state = EXPLORE;
                return MOVE;
            }
            return STOP;
        }
        
        case BACKTRACK:
            // Simple backtracking - just turn right and try a different direction
            turtle_data.facing = static_cast<Orientation>((turtle_data.facing + 1) % ORIENTATION_COUNT);
            *cur_state = EXPLORE;
            return TURNRIGHT;
            
        case AT_END:
        default:
            return STOP;
    }
}
