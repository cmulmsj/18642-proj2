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

static int8_t visit_count[GRID_SIZE][GRID_SIZE] = {0};

void logVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        visit_count[x][y]++;
    }
}

uint8_t getVisitCount(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        return visit_count[x][y];
    }
    return 0;
}

TurtleCommand decideTurtleAction(bool obstacle_detected, bool goal_reached, NavigationMode* current_mode) {
    static int rotation_count = 0;

    if (goal_reached) {
        *current_mode = NavigationMode::COMPLETE;
        return TurtleCommand::HALT;
    }

    switch (*current_mode) {
        case NavigationMode::INITIAL:
        case NavigationMode::FORWARD:
            if (obstacle_detected) {
                *current_mode = NavigationMode::ADJUST;
                rotation_count = 1;
                return TurtleCommand::ROTATE_CW;
            } else {
                *current_mode = NavigationMode::FORWARD;
                return TurtleCommand::ADVANCE;
            }

        case NavigationMode::ADJUST:
            if (obstacle_detected) {
                rotation_count++;
                if (rotation_count >= DIRECTION_COUNT) {
                    rotation_count = 0;
                    return TurtleCommand::ROTATE_CCW;
                }
                return TurtleCommand::ROTATE_CW;
            } else {
                *current_mode = NavigationMode::FORWARD;
                return TurtleCommand::ADVANCE;
            }

        default:
            ROS_ERROR("Invalid Navigation Mode");
            return TurtleCommand::HALT;
    }
}
