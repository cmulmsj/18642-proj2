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

static int8_t visited[MAZE_SIZE][MAZE_SIZE] = {0};

void record_visited(QPointF& pos_) {
    visited[static_cast<int>(pos_.x())][static_cast<int>(pos_.y())]++;
}

uint8_t get_visited(QPointF& pos_) {
    return visited[static_cast<int>(pos_.x())][static_cast<int>(pos_.y())];
}

turtleMove studentTurtleStep(bool bumped, bool goal, State* cur_state) {
    turtleMove nextMove;

    switch (*cur_state) {
        case INIT:
        case GO:
            if (goal) {
                *cur_state = GOAL;
                nextMove = STOP;
            } else {
                *cur_state = TURN;
                nextMove = TURNRIGHT;
            }
            break;

        case TURN:
            if (bumped) {
                *cur_state = TURN;
                nextMove = TURNLEFT;
            } else {
                *cur_state = GO;
                nextMove = MOVE;
            }
            break;

        case GOAL:
            nextMove = STOP;
            break;

        default:
            ROS_ERROR("Invalid State");
            nextMove = STOP;
            break;
    }

    return nextMove;
}

