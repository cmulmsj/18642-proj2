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
    static int cooldown = 0;
    static NavigationMode current_mode = NavigationMode::INITIAL;

    if (cooldown == 0) {
        if (current_mode == NavigationMode::INITIAL) {
            logVisit(pos_);
            displayVisits(getVisitCount(pos_));
        }

        bool obstacle_detected = detectObstacle(pos_, nw_or);
        bool goal_reached = atend(pos_.x(), pos_.y());

        TurtleCommand next_command = decideTurtleAction(obstacle_detected, goal_reached, &current_mode);
        nw_or = updateOrientation(nw_or, next_command);

        ROS_INFO("Facing=%d  Mode=%d  Command=%d", nw_or, static_cast<int>(current_mode), static_cast<int>(next_command));

        if (next_command == TurtleCommand::ADVANCE && !obstacle_detected) {
            pos_ = updatePosition(pos_, nw_or);
            logVisit(pos_);
            displayVisits(getVisitCount(pos_));
        }

        if (current_mode == NavigationMode::COMPLETE) {
            return false;
        }

        cooldown = MOVE_DELAY;
        return true;
    }

    cooldown--;
    return false;
}

QPointF updatePosition(QPointF pos_, int orientation) {
    switch (static_cast<TurtleDirection>(orientation)) {
        case TurtleDirection::WEST:  pos_.setX(pos_.x() - 1); break;
        case TurtleDirection::SOUTH: pos_.setY(pos_.y() + 1); break;
        case TurtleDirection::EAST:  pos_.setX(pos_.x() + 1); break;
        case TurtleDirection::NORTH: pos_.setY(pos_.y() - 1); break;
    }
    return pos_;
}

int updateOrientation(int orientation, TurtleCommand next_command) {
    switch (next_command) {
        case TurtleCommand::ROTATE_CW:  return (orientation + 1) % DIRECTION_COUNT;
        case TurtleCommand::ROTATE_CCW: return (orientation - 1 + DIRECTION_COUNT) % DIRECTION_COUNT;
        default: return orientation;
    }
}

bool detectObstacle(QPointF pos_, int facing) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (static_cast<TurtleDirection>(facing)) {
        case TurtleDirection::WEST:  x2--; break;
        case TurtleDirection::SOUTH: y2++; break;
        case TurtleDirection::EAST:  x2++; break;
        case TurtleDirection::NORTH: y2--; break;
    }

    return bumped(x1, y1, x2, y2);
}
