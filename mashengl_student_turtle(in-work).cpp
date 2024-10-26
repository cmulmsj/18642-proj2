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

static DirectionInfo adjacent_cells[ORIENTATION_COUNT];  // Indexed by Orientation enum
static Orientation current_orientation = NORTH;

// Helper function to get position of adjacent cell in given orientation
static QPointF getAdjacentPosition(QPointF current_pos, Orientation orient) {
    QPointF adj_pos = current_pos;
    switch(orient) {
        case NORTH: adj_pos.setY(adj_pos.y() + 1); break;
        case EAST:  adj_pos.setX(adj_pos.x() + 1); break;
        case SOUTH: adj_pos.setY(adj_pos.y() - 1); break;
        case WEST:  adj_pos.setX(adj_pos.x() - 1); break;
        default: break;
    }
    return adj_pos;
}

// C1: Implementation of findLowestVisitCount compute function
static Orientation findLowestVisitCount() {
    uint8_t min_count = UINT8_MAX;
    Orientation selected_orient = current_orientation;
    
    // First pass: look for unvisited cells
    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        Orientation orient = static_cast<Orientation>(i);
        if (adjacent_cells[i].accessible && adjacent_cells[i].visit_count == 0) {
            // Prioritize forward direction if unvisited
            if (orient == current_orientation) {
                return orient;
            }
            if (adjacent_cells[i].visit_count < min_count) {
                min_count = adjacent_cells[i].visit_count;
                selected_orient = orient;
            }
        }
    }

    // Second pass: if no unvisited cells, find lowest visit count
    if (min_count == UINT8_MAX) {
        for (int i = 0; i < ORIENTATION_COUNT; i++) {
            if (adjacent_cells[i].accessible && adjacent_cells[i].visit_count < min_count) {
                min_count = adjacent_cells[i].visit_count;
                selected_orient = static_cast<Orientation>(i);
            }
        }
    }

    return selected_orient;
}

// C2: Implementation of getRequiredRotation compute function
static int getRequiredRotation(Orientation target_orientation) {
    int diff = (target_orientation - current_orientation + ORIENTATION_COUNT) % ORIENTATION_COUNT;
    if (diff > 2) {
        return diff - ORIENTATION_COUNT;  // Turn left
    }
    return diff;  // Turn right
}

// C3: Implementation of selectLowestVisitCount compute function
static Orientation selectLowestVisitCount() {
    uint8_t min_count = UINT8_MAX;
    Orientation backtrack_orient = current_orientation;

    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        if (adjacent_cells[i].accessible && adjacent_cells[i].visit_count < min_count) {
            min_count = adjacent_cells[i].visit_count;
            backtrack_orient = static_cast<Orientation>(i);
        }
    }

    return backtrack_orient;
}

turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {
    static turtleMove next_move = STOPPING;
    static TurtleState internal_state = INITIALIZING;
    static Orientation target_orientation = NORTH;
    static int remaining_rotation = 0;
    
    ROS_INFO("studentTurtleStep: Current State: %d, Bumped: %s, Goal: %s",
             internal_state, bumped ? "Yes" : "No", goal ? "Yes" : "No");

    // Update adjacent cells information
    QPointF current_pos(0, 0);  // Will be set by maze.cpp
    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        Orientation orient = static_cast<Orientation>(i);
        QPointF adj_pos = getAdjacentPosition(current_pos, orient);
        adjacent_cells[i].accessible = !detectObstacle(current_pos, orient);
        adjacent_cells[i].visit_count = retrieveVisitCount(adj_pos);
    }

    // Main state machine
    switch (internal_state) {
        case EVALUATING: {  // S1: EVALUATING state
            ROS_INFO("State S1: EVALUATING");
            next_move = STOPPING;

            Orientation nextOrient = findLowestVisitCount();
            if (nextOrient != current_orientation) {
                target_orientation = nextOrient;
                remaining_rotation = getRequiredRotation(target_orientation);
                
                // T1: Transition to ROTATING if rotation needed
                if (remaining_rotation != 0) {
                    internal_state = ROTATING;
                    next_move = (remaining_rotation > 0) ? TURNING_RIGHT : TURNING_LEFT;
                    ROS_INFO("Transition T1: EVALUATING -> ROTATING");
                }
                // T2: Transition to MOVING if no rotation needed
                else {
                    internal_state = MOVING;
                    next_move = MOVING;
                    ROS_INFO("Transition T2: EVALUATING -> MOVING");
                }
            }
            // T3: Transition to BACKTRACKING if needed
            else if (!adjacent_cells[nextOrient].accessible) {
                internal_state = BACKTRACKING;
                next_move = STOPPING;
                ROS_INFO("Transition T3: EVALUATING -> BACKTRACKING");
            }
            else {
                internal_state = MOVING;
                next_move = MOVING;
                ROS_INFO("Moving forward in current direction");
            }
            break;
        }

        case ROTATING: {  // S2: ROTATING state
            ROS_INFO("State S2: ROTATING");
            if (remaining_rotation != 0) {
                next_move = (remaining_rotation > 0) ? TURNING_RIGHT : TURNING_LEFT;
                remaining_rotation += (remaining_rotation > 0) ? -1 : 1;
                current_orientation = static_cast<Orientation>((current_orientation + 
                    (remaining_rotation > 0 ? 1 : 3)) % ORIENTATION_COUNT);
            }
            // T4: Transition back to EVALUATING when rotation complete
            else {
                internal_state = EVALUATING;
                next_move = STOPPING;
                ROS_INFO("Transition T4: ROTATING -> EVALUATING");
            }
            break;
        }

        case MOVING: {  // S3: MOVING state
            ROS_INFO("State S3: MOVING");
            if (bumped) {
                internal_state = EVALUATING;
                next_move = STOPPING;
                ROS_INFO("Hit wall, returning to EVALUATING");
            }
            else if (goal) {  // T6: Transition to GOAL_REACHED
                internal_state = GOAL_REACHED;
                next_move = STOPPING;
                ROS_INFO("Transition T6: MOVING -> GOAL_REACHED");
            }
            else {  // T5: Transition back to EVALUATING after move
                internal_state = EVALUATING;
                next_move = MOVING;
                ROS_INFO("Transition T5: MOVING -> EVALUATING");
            }
            break;
        }

        case BACKTRACKING: {  // S4: BACKTRACKING state
            ROS_INFO("State S4: BACKTRACKING");
            Orientation backtrackOrient = selectLowestVisitCount();
            if (backtrackOrient != current_orientation) {
                target_orientation = backtrackOrient;
                // T7: Transition back to EVALUATING for rotation/movement
                internal_state = EVALUATING;
                next_move = STOPPING;
                ROS_INFO("Transition T7: BACKTRACKING -> EVALUATING");
            }
            break;
        }

        case GOAL_REACHED: {  // S5: GOAL_REACHED state
            ROS_INFO("State S5: GOAL_REACHED");
            next_move = STOPPING;
            break;
        }

        default: {
            ROS_ERROR("Invalid state, resetting to EVALUATING");
            internal_state = EVALUATING;
            next_move = STOPPING;
            break;
        }
    }

    *cur_state = internal_state;
    ROS_INFO("Next move: %d", next_move);
    return next_move;
}
