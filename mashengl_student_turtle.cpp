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

static int8_t visit_record[MAZE_GRID_SIZE][MAZE_GRID_SIZE] = {0};

void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());

    if (x < 0 || x >= MAZE_GRID_SIZE || y < 0 || y >= MAZE_GRID_SIZE) {
        ROS_ERROR("Attempted to add visit outside maze boundaries: (%d, %d)", x, y);
        return;
    }

    visit_record[x][y]++;
    ROS_DEBUG("Recorded visit at (%d, %d). Total visits: %d", x, y, visit_record[x][y]);
}

uint8_t retrieveVisitCount(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());

    if (x < 0 || x >= MAZE_GRID_SIZE || y < 0 || y >= MAZE_GRID_SIZE) {
        ROS_ERROR("Attempted to retrieve visit count outside maze boundaries: (%d, %d)", x, y);
        return 0;
    }

    ROS_DEBUG("Retrieved visit count at (%d, %d): %d", x, y, visit_record[x][y]);
    return visit_record[x][y];
}

turtleMove studentTurtleStep(bool bumped, bool goal, TurtleState* cur_state) {
    turtleMove nextMove;

    ROS_INFO("Current State: %d, Obstacle: %s, Goal: %s",
             *cur_state, bumped ? "Yes" : "No", goal ? "Yes" : "No");

    switch (*cur_state) {
        case INIT:
        case GO:
            if (goal) {
                *cur_state = GOAL;
                nextMove = STOPPING;
                ROS_INFO("Goal reached. Transitioning to GOAL state.");
            } else {
                *cur_state = TURN;
                nextMove = TURNING_RIGHT;
                ROS_INFO("Transitioning to TURN state. Next move: TURNING_RIGHT.");
            }
            break;

        case TURN:
            if (bumped) {
                *cur_state = TURN;
                nextMove = TURNING_LEFT;
                ROS_INFO("Obstacle detected while turning. Next move: TURNING_LEFT.");
            } else {
                *cur_state = GO;
                nextMove = MOVING;
                ROS_INFO("No obstacle detected. Transitioning to GO state. Next move: MOVING.");
            }
            break;

        case GOAL:
            nextMove = STOPPING;
            ROS_INFO("Already in GOAL state. Next move: STOPPING.");
            break;

        default:
            ROS_ERROR("Invalid TurtleState encountered: %d", *cur_state);
            nextMove = STOPPING;
            break;
    }

    ROS_INFO("Selected Next Move: %d", nextMove);
    return nextMove;
}


