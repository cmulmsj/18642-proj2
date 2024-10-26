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
    static Orientation target_orientation = NORTH;
    static int remaining_rotation = 0;
    
    ROS_INFO("studentTurtleStep: Current State: %d, Bumped: %s, Goal: %s",
             *cur_state, bumped ? "Yes" : "No", goal ? "Yes" : "No");

    // Update adjacent cells information
    QPointF current_pos(0, 0);  // Will be set by maze.cpp
    for (int i = 0; i < ORIENTATION_COUNT; i++) {
        Orientation orient = static_cast<Orientation>(i);
        QPointF adj_pos = getAdjacentPosition(current_pos, orient);
        adjacent_cells[i].accessible = !detectObstacle(current_pos, orient);
        adjacent_cells[i].visit_count = retrieveVisitCount(adj_pos);
    }

    // Main state machine
    switch (*cur_state) {
        case INITIALIZING: {  // S1: Initial evaluation state
            ROS_INFO("State S1: INITIALIZING");
            
            Orientation nextOrient = findLowestVisitCount();
            if (goal) {
                *cur_state = GOAL_REACHED;
                next_move = STOPPING;
                ROS_INFO("Goal reached from INITIALIZING");
            } else if (nextOrient != current_orientation) {
                *cur_state = TURNING_STATE;
                next_move = TURNING_RIGHT;
                target_orientation = nextOrient;
                ROS_INFO("Transition to TURNING_STATE for orientation change");
            } else if (!adjacent_cells[nextOrient].accessible) {
                next_move = TURNING_RIGHT;
                ROS_INFO("Obstacle detected, turning right");
            } else {
                *cur_state = MOVING_STATE;
                next_move = MOVING;
                ROS_INFO("Moving forward in current direction");
            }
            break;
        }

        case TURNING_STATE: {  // S2: Turning state
            ROS_INFO("State S2: TURNING_STATE");
            if (bumped) {
                next_move = TURNING_LEFT;
                ROS_INFO("Obstacle detected while turning, turning left");
            } else {
                *cur_state = MOVING_STATE;
                next_move = MOVING;
                ROS_INFO("Clear path after turn, moving forward");
            }
            break;
        }

        case MOVING_STATE: {  // S3: Moving state
            ROS_INFO("State S3: MOVING_STATE");
            if (goal) {
                *cur_state = GOAL_REACHED;
                next_move = STOPPING;
                ROS_INFO("Goal reached while moving");
            } else {
                *cur_state = TURNING_STATE;
                next_move = TURNING_RIGHT;
                ROS_INFO("Completed move, evaluating next direction");
            }
            break;
        }

        case GOAL_REACHED: {  // S4: Goal reached state
            ROS_INFO("State S4: GOAL_REACHED");
            next_move = STOPPING;
            break;
        }

        default: {
            ROS_ERROR("Invalid state encountered: %d", *cur_state);
            *cur_state = INITIALIZING;
            next_move = STOPPING;
            break;
        }
    }

    ROS_INFO("Next move: %d", next_move);
    return next_move;
}
