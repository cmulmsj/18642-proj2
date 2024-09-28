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
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visited[x][y]++;
    }
}

uint8_t get_visited(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visited[x][y];
    }
    return 0;
}

turtleMove studentTurtleStep(bool bumped, bool goal, State* cur_state) {
    static int turns = 0;

    if (goal) {
        *cur_state = FINISH;
        return STOP;
    }

    switch (*cur_state) {
        case EXPLORE:
            if (!bumped) {
                turns = 0;
                return FORWARD;
            } else {
                *cur_state = ROTATE;
                turns = 1;
                return TURN_RIGHT;
            }

        case ROTATE:
            if (bumped) {
                turns++;
                if (turns >= 4) {
                    *cur_state = BACKTRACK;
                    return TURN_LEFT;
                }
                return TURN_RIGHT;
            } else {
                *cur_state = EXPLORE;
                return FORWARD;
            }

        case BACKTRACK:
            if (!bumped) {
                *cur_state = EXPLORE;
                turns = 0;
                return FORWARD;
            } else {
                return TURN_LEFT;
            }

        default:
            ROS_ERROR("Invalid State");
            return STOP;
    }
}
