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

static int8_t visit_record[GRID_SIZE][GRID_SIZE] = {0};

void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        visit_record[x][y]++;
    }
}

uint8_t getVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        return visit_record[x][y];
    }
    return 0;
}

TurtleCommand studentTurtleStep(bool bumped, bool goal, NavigationMode* cur_state) {
    TurtleCommand nextMove;

    switch (*cur_state) {
        case NavigationMode::INITIAL:
        case NavigationMode::FORWARD:
            if (goal) {
                *cur_state = NavigationMode::COMPLETE;
                nextMove = TurtleCommand::HALT;
            } else {
                *cur_state = NavigationMode::ADJUST;
                nextMove = TurtleCommand::ROTATE_CW;
            }
            break;

        case NavigationMode::ADJUST:
            if (bumped) {
                nextMove = TurtleCommand::ROTATE_CCW;
            } else {
                *cur_state = NavigationMode::FORWARD;
                nextMove = TurtleCommand::ADVANCE;
            }
            break;

        case NavigationMode::COMPLETE:
            nextMove = TurtleCommand::HALT;
            break;

        default:
            ROS_ERROR("Invalid Navigation Mode");
            nextMove = TurtleCommand::HALT;
            break;
    }

    return nextMove;
}

