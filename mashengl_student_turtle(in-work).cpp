/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
 */

#include "student.h"

// Internal helper struct to track directions and their visit counts
struct DirectionInfo {
    bool accessible;
    uint8_t visit_count;
};

static DirectionInfo adjacent_cells[4];  // Indexed by Direction enum
static Direction current_direction = NORTH;

// Helper function to get position of adjacent cell in given direction
static QPointF getAdjacentPosition(QPointF current_pos, Direction dir) {
    QPointF adj_pos = current_pos;
    switch(dir) {
        case NORTH: adj_pos.setY(adj_pos.y() + 1); break;
        case EAST:  adj_pos.setX(adj_pos.x() + 1); break;
        case SOUTH: adj_pos.setY(adj_pos.y() - 1); break;
        case WEST:  adj_pos.setX(adj_pos.x() - 1); break;
    }
    return adj_pos;
}

// C1: Implementation of findLowestVisitCount compute function
static Direction findLowestVisitCount() {
    uint8_t min_count = UINT8_MAX;
    Direction selected_dir = current_direction;
    
    // First pass: look for unvisited cells
    for (int dir = 0; dir < 4; dir++) {
        if (adjacent_cells[dir].accessible && adjacent_cells[dir].visit_count == 0) {
            // Prioritize forward direction if unvisited
            if (dir == current_direction) {
                return static_cast<Direction>(dir);
            }
            if (adjacent_cells[dir].visit_count < min_count) {
                min_count = adjacent_cells[dir].visit_count;
                selected_dir = static_cast<Direction>(dir);
            }
        }
    }

    // Second pass: if no unvisited cells, find lowest visit count
    if (min_count == UINT8_MAX) {
        for (int dir = 0; dir < 4; dir++) {
            if (adjacent_cells[dir].accessible && adjacent_cells[dir].visit_count < min_count) {
                min_count = adjacent_cells[dir].visit_count;
                selected_dir = static_cast<Direction>(dir);
            }
        }
    }

    return selected_dir;
}

// C2: Implementation of getRequiredRotation compute function
static int getRequiredRotation(Direction target_direction) {
    int diff = (target_direction - current_direction + 4) % 4;
    if (diff > 2) {
        return diff - 4;  // Turn left
    }
    return diff;  // Turn right
}

// C3: Implementation of selectLowestVisitCount compute function
static Direction selectLowestVisitCount() {
    uint8_t min_count = UINT8_MAX;
    Direction backtrack_dir = current_direction;

    // Find accessible cell with lowest visit count
    for (int dir = 0; dir < 4; dir++) {
        if (adjacent_cells[dir].accessible && adjacent_cells[dir].visit_count < min_count) {
            min_count = adjacent_cells[dir].visit_count;
            backtrack_dir = static_cast<Direction>(dir);
        }
    }

    return backtrack_dir;
}

turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {
    static turtleMove next_move = STOPPING;
    static TurtleState internal_state = INITIALIZING;
    static Direction target_direction = NORTH;
    static int remaining_rotation = 0;

    // Update adjacent cells information
    QPointF current_pos(0, 0);  // Will be set by maze.cpp
    for (int dir = 0; dir < 4; dir++) {
        QPointF adj_pos = getAdjacentPosition(current_pos, static_cast<Direction>(dir));
        adjacent_cells[dir].accessible = !detectObstacle(current_pos, static_cast<Orientation>(dir));
        adjacent_cells[dir].visit_count = retrieveVisitCount(adj_pos);
    }

    // Main state machine
    switch (internal_state) {
        case EVALUATING: {  // S1: EVALUATING state
            next_move = STOPPING;

            Direction nextDir = findLowestVisitCount();
            if (nextDir != current_direction) {
                target_direction = nextDir;
                remaining_rotation = getRequiredRotation(target_direction);
                
                // T1: Transition to ROTATING if rotation needed
                if (remaining_rotation != 0) {
                    internal_state = ROTATING;
                    next_move = (remaining_rotation > 0) ? TURNING_RIGHT : TURNING_LEFT;
                }
                // T2: Transition to MOVING if no rotation needed
                else {
                    internal_state = MOVING;
                    next_move = MOVING;
                }
            }
            // T3: Transition to BACKTRACKING if needed
            else if (!adjacent_cells[nextDir].accessible) {
                internal_state = BACKTRACKING;
                next_move = STOPPING;
            }
            else {
                internal_state = MOVING;
                next_move = MOVING;
            }
            break;
        }

        case ROTATING: {  // S2: ROTATING state
            if (remaining_rotation != 0) {
                next_move = (remaining_rotation > 0) ? TURNING_RIGHT : TURNING_LEFT;
                remaining_rotation += (remaining_rotation > 0) ? -1 : 1;
                current_direction = static_cast<Direction>((current_direction + 
                    (remaining_rotation > 0 ? 1 : 3)) % 4);
            }
            // T4: Transition back to EVALUATING when rotation complete
            else {
                internal_state = EVALUATING;
                next_move = STOPPING;
            }
            break;
        }

        case MOVING: {  // S3: MOVING state
            if (bumped) {
                internal_state = EVALUATING;
                next_move = STOPPING;
            }
            else if (goal) {  // T6: Transition to GOAL_REACHED
                internal_state = GOAL_REACHED;
                next_move = STOPPING;
            }
            else {  // T5: Transition back to EVALUATING after move
                internal_state = EVALUATING;
                next_move = MOVING;
            }
            break;
        }

        case BACKTRACKING: {  // S4: BACKTRACKING state
            Direction backtrackDir = selectLowestVisitCount();
            if (backtrackDir != current_direction) {
                target_direction = backtrackDir;
                // T7: Transition back to EVALUATING for rotation/movement
                internal_state = EVALUATING;
                next_move = STOPPING;
            }
            break;
        }

        case GOAL_REACHED: {  // S5: GOAL_REACHED state
            next_move = STOPPING;
            break;
        }

        default: {
            internal_state = EVALUATING;
            next_move = STOPPING;
            break;
        }
    }

    *cur_state = internal_state;
    return next_move;
}
