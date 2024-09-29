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
    if (goal) {
        *cur_state = NavigationMode::COMPLETE;
        return TurtleCommand::HALT;
    }

    switch (*cur_state) {
        case NavigationMode::INITIAL:
        case NavigationMode::FORWARD:
            if (!bumped) {
                // If there's no wall to the right, turn right
                *cur_state = NavigationMode::FORWARD;
                return TurtleCommand::ROTATE_CW;
            } else {
                // If there's a wall to the right, move forward
                *cur_state = NavigationMode::FORWARD;
                return TurtleCommand::ADVANCE;
            }

        case NavigationMode::ADJUST:
            if (bumped) {
                // If we bump into a wall while moving forward, turn left
                *cur_state = NavigationMode::FORWARD;
                return TurtleCommand::ROTATE_CCW;
            } else {
                // This state should not be reached in normal operation
                *cur_state = NavigationMode::FORWARD;
                return TurtleCommand::ADVANCE;
            }

        default:
            ROS_ERROR("Invalid Navigation Mode");
            return TurtleCommand::HALT;
    }
}
