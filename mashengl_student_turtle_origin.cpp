/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
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
        ROS_ERROR("addVisit: Attempted to add visit outside maze boundaries: (%d, %d)", x, y);
        return;
    }

    visit_record[x][y]++;
    ROS_DEBUG("addVisit: Recorded visit at (%d, %d). Total visits: %d", x, y, visit_record[x][y]);
}

uint8_t retrieveVisitCount(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());

    if (x < 0 || x >= MAZE_GRID_SIZE || y < 0 || y >= MAZE_GRID_SIZE) {
        ROS_ERROR("retrieveVisitCount: Attempted to retrieve visit count outside maze boundaries: (%d, %d)", x, y);
        return 0;
    }

    ROS_DEBUG("retrieveVisitCount: Retrieved visit count at (%d, %d): %d", x, y, visit_record[x][y]);
    return visit_record[x][y];
}

turtleMove studentTurtleStep(bool obstacle_detected, bool goal, TurtleState* cur_state) {
    turtleMove nextMove;

    ROS_INFO("studentTurtleStep: Current State: %d, Obstacle: %s, Goal: %s",
             *cur_state, obstacle_detected ? "Yes" : "No", goal ? "Yes" : "No");

    switch (*cur_state) {
        case INITIALIZING:
        case MOVING_STATE:
            if (goal) {
                *cur_state = GOAL_REACHED;
                nextMove = STOPPING;
                ROS_INFO("studentTurtleStep: Goal reached. Transitioning to GOAL_REACHED state.");
            } else {
                *cur_state = TURNING_STATE;
                nextMove = TURNING_RIGHT;
                ROS_INFO("studentTurtleStep: Transitioning to TURNING_STATE. Next move: TURNING_RIGHT.");
            }
            break;

        case TURNING_STATE:
            if (obstacle_detected) {
                *cur_state = TURNING_STATE;
                nextMove = TURNING_LEFT;
                ROS_INFO("studentTurtleStep: Obstacle detected while turning. Next move: TURNING_LEFT.");
            } else {
                *cur_state = MOVING_STATE;
                nextMove = MOVING;
                ROS_INFO("studentTurtleStep: No obstacle detected. Transitioning to MOVING_STATE. Next move: MOVING.");
            }
            break;

        case GOAL_REACHED:
            nextMove = STOPPING;
            ROS_INFO("studentTurtleStep: Already in GOAL_REACHED state. Next move: STOPPING.");
            break;

        default:
            ROS_ERROR("studentTurtleStep: Invalid TurtleState encountered: %d", *cur_state);
            nextMove = STOPPING;
            break;
    }

    ROS_INFO("studentTurtleStep: Selected Next Move: %d", nextMove);
    return nextMove;
}


