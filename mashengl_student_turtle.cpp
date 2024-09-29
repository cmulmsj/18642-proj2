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

static int8_t visited[GRID_DIMENSION][GRID_DIMENSION] = {0};

void logVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_DIMENSION && y >= 0 && y < GRID_DIMENSION) {
        visited[x][y]++;
    }
}

uint8_t getVisitCount(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_DIMENSION && y >= 0 && y < GRID_DIMENSION) {
        return visited[x][y];
    }
    return 0;
}

TurtleAction studentTurtleStep(bool wall_ahead, bool at_goal, TurtleState* current_state) {
    static int turn_count = 0;

    if (at_goal) {
        *current_state = TurtleState::FINISHED;
        return TurtleAction::STOP;
    }

    switch (*current_state) {
        case TurtleState::EXPLORING:
            if (!wall_ahead) {
                turn_count = 0;
                return TurtleAction::MOVE;
            } else {
                *current_state = TurtleState::TURNING;
                turn_count = 1;
                return TurtleAction::TURN_RIGHT;
            }

        case TurtleState::TURNING:
            if (wall_ahead) {
                turn_count++;
                if (turn_count >= FULL_ROTATION) {
                    *current_state = TurtleState::BACKTRACKING;
                    turn_count = 0;
                    return TurtleAction::TURN_LEFT;
                }
                return TurtleAction::TURN_RIGHT;
            } else {
                *current_state = TurtleState::EXPLORING;
                return TurtleAction::MOVE;
            }

        case TurtleState::BACKTRACKING:
            if (!wall_ahead) {
                *current_state = TurtleState::EXPLORING;
                return TurtleAction::MOVE;
            } else {
                return TurtleAction::TURN_LEFT;
            }

        default:
            ROS_ERROR("Invalid Turtle State");
            return TurtleAction::STOP;
    }
}
