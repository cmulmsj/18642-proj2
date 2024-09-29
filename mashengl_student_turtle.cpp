/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Initialize the visit map to keep track of visited cells
static int8_t visit_map[MAZE_SIZE][MAZE_SIZE] = {0};

// Renamed to addVisit
void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visit_map[x][y]++;
        ROS_INFO("addVisit: Visited (%d, %d). Visit count: %d", x, y, visit_map[x][y]);
    } else {
        ROS_WARN("addVisit: Attempted to visit out-of-bounds position (%d, %d). Ignoring.", x, y);
    }
}

// Renamed to getVisit
uint8_t getVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        ROS_INFO("getVisit: Visit count at (%d, %d): %d", x, y, visit_map[x][y]);
        return visit_map[x][y];
    }
    ROS_WARN("getVisit: Attempted to get visit count for out-of-bounds position (%d, %d). Returning 0.", x, y);
    return 0;
}

turtleMove studentTurtleStep(bool bumped, bool goal, State* cur_state) {
    turtleMove nextMove;

    // Log the current state and sensor inputs
    ROS_INFO("studentTurtleStep called with - Bumped: %d, Goal: %d, Current State: %d",
             bumped, goal, *cur_state);

    if (goal) {
        *cur_state = GOAL;
        nextMove = STOP;
        ROS_INFO("studentTurtleStep: Goal reached. Transitioning to GOAL state. Next Move: STOP");
        return nextMove;
    }

    switch (*cur_state) {
        case INIT:
        case GO:
            if (!bumped) {
                *cur_state = GO;
                nextMove = MOVE;
                ROS_INFO("studentTurtleStep: State %d: Path is clear. Advancing forward. Next Move: MOVE", *cur_state);
            } else {
                *cur_state = TURN;
                nextMove = TURNRIGHT;
                ROS_INFO("studentTurtleStep: State %d: Path is blocked. Turning right. Next Move: TURNRIGHT", *cur_state);
            }
            break;

        case TURN:
            if (bumped) {
                *cur_state = TURN;
                nextMove = TURNLEFT;
                ROS_WARN("studentTurtleStep: State TURN: Still blocked after turning right. Turning left. Next Move: TURNLEFT");
            } else {
                *cur_state = GO;
                nextMove = MOVE;
                ROS_INFO("studentTurtleStep: State TURN: Path is now clear after turning. Advancing forward. Next Move: MOVE");
            }
            break;

        case GOAL:
            nextMove = STOP;
            ROS_INFO("studentTurtleStep: State GOAL: Halting the turtle.");
            break;

        default:
            ROS_ERROR("studentTurtleStep: Encountered invalid state %d. Halting.", *cur_state);
            nextMove = STOP;
            break;
    }

    ROS_INFO("studentTurtleStep: Transitioned to state %d with next move %d", *cur_state, nextMove);
    return nextMove;
}

