/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    static int cooldown_timer = 0;
    static NavigationPhase current_phase = NavigationPhase::SCOUTING;

    if (cooldown_timer == 0) {
        logVisit(pos_);
        displayVisits(getVisitCount(pos_));

        bool obstacle_detected = detectObstacle(pos_, nw_or);
        bool goal_reached = atend(pos_.x(), pos_.y());

        TurtleAction next_action = studentTurtleStep(obstacle_detected, goal_reached, &current_phase);
        
        ROS_INFO("Position: (%.0f, %.0f), Facing: %d, Action: %d, Phase: %d", 
                 pos_.x(), pos_.y(), nw_or, static_cast<int>(next_action), static_cast<int>(current_phase));

        if (next_action == TurtleAction::PROCEED && !obstacle_detected) {
            pos_ = translatePos(pos_, nw_or);
        } else if (next_action != TurtleAction::HALT) {
            nw_or = translateOrnt(nw_or, next_action);
        }

        if (current_phase == NavigationPhase::MISSION_COMPLETE) {
            return false;
        }

        cooldown_timer = MOVE_COOLDOWN;
        return true;
    }

    cooldown_timer--;
    return false;
}

QPointF translatePos(QPointF pos_, int orientation) {
    switch (static_cast<MazeDirection>(orientation)) {
        case MazeDirection::WESTWARD:  pos_.setX(pos_.x() - 1); break;
        case MazeDirection::SOUTHWARD: pos_.setY(pos_.y() + 1); break;
        case MazeDirection::EASTWARD:  pos_.setX(pos_.x() + 1); break;
        case MazeDirection::NORTHWARD: pos_.setY(pos_.y() - 1); break;
    }
    return pos_;
}

int translateOrnt(int orientation, TurtleAction next_action) {
    if (next_action == TurtleAction::PIVOT_CLOCKWISE) {
        return (orientation + 1) % ORIENTATION_COUNT;
    } else if (next_action == TurtleAction::PIVOT_COUNTERCLOCKWISE) {
        return (orientation + ORIENTATION_COUNT - 1) % ORIENTATION_COUNT;
    }
    return orientation;
}

bool detectObstacle(QPointF pos_, int facing) {
    QPointF next_pos = translatePos(pos_, facing);
    return bumped(pos_.x(), pos_.y(), next_pos.x(), next_pos.y());
}


