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

static int8_t visit_map[MAZE_SIZE][MAZE_SIZE] = {0};

// Renamed to addVisit as per your convention
void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visit_map[x][y]++;
    }
}

// Renamed to getVisit as per your convention
uint8_t getVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visit_map[x][y];
    }
    return 0;
}

// Slightly modified logic for state transitions and movement
TurtleMove studentTurtleStep(bool bumped, bool goal, NavigationMode* cur_state) {
    TurtleMove nextMove;

    switch (*cur_state) {
        case INITIAL:
        case PROCEED:
            if (goal) {
                *cur_state = COMPLETE;
                nextMove = HALT;
            } else {
                *cur_state = ADJUST;
                nextMove = ROTATE_CW; // Start by rotating clockwise to explore
            }
            break;

        case ADJUST:
            if (bumped) {
                nextMove = ROTATE_CCW; // Rotate counterclockwise if a wall is hit
            } else {
                *cur_state = PROCEED;
                nextMove = ADVANCE; // Move forward if the path is clear
            }
            break;

        case COMPLETE:
            nextMove = HALT;
            break;

        default:
            ROS_ERROR("Invalid Navigation Mode");
            nextMove = HALT;
            break;
    }

    return nextMove;
}

