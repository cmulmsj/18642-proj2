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

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    static int count_down = 0;
    static State current_state = INIT;

    if (count_down == 0) {
        if (current_state == INIT) {
            record_visited(pos_);
            displayVisits(get_visited(pos_));
        }

        bool has_wall = check_bumped(pos_, nw_or);
        bool at_goal = atend(pos_.x(), pos_.y());

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("Orientation=%d  State=%d  NextMove=%d", nw_or, current_state, nextMove);

        if (nextMove == MOVE) {
            pos_ = translatePos(pos_, nw_or);
            record_visited(pos_);
            displayVisits(get_visited(pos_));
        }

        if (current_state == GOAL) {
            return false;
        }

        count_down = TIMEOUT;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, int orientation) {
    switch (static_cast<Orientation>(orientation)) {
        case LEFT:  pos_.setX(pos_.x() - 1); break;
        case DOWN:  pos_.setY(pos_.y() + 1); break;
        case RIGHT: pos_.setX(pos_.x() + 1); break;
        case UP:    pos_.setY(pos_.y() - 1); break;
        default:    ROS_ERROR("Invalid Orientation"); break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case TURNRIGHT: return (orientation + 1) % NUM_ORIENTATIONS;
        case TURNLEFT:  return (orientation - 1 + NUM_ORIENTATIONS) % NUM_ORIENTATIONS;
        case MOVE:
        case STOP:      return orientation;
        default:        ROS_ERROR("Invalid Move"); return orientation;
    }
}

bool check_bumped(QPointF pos_, int orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (static_cast<Orientation>(orient)) {
        case LEFT:  x2--; break;
        case DOWN:  y2++; break;
        case RIGHT: x2++; break;
        case UP:    y2--; break;
        default:    ROS_ERROR("Invalid Orientation"); return false;
    }

    return bumped(x1, y1, x2, y2);
}
