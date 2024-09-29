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
    static TurtleState current_state = TurtleState::EXPLORING;

    if (cooldown_timer == 0) {
        logVisit(pos_);
        displayVisits(getVisitCount(pos_));

        bool wall_ahead = detectObstacle(pos_, nw_or);
        bool at_goal = atend(pos_.x(), pos_.y());

        TurtleAction next_action = studentTurtleStep(wall_ahead, at_goal, &current_state);
        
        ROS_INFO("Position: (%.0f, %.0f), Facing: %d, Action: %d, State: %d", 
                 pos_.x(), pos_.y(), nw_or, static_cast<int>(next_action), static_cast<int>(current_state));

        if (next_action == TurtleAction::MOVE && !wall_ahead) {
            pos_ = translatePos(pos_, nw_or);
        } else if (next_action != TurtleAction::STOP) {
            nw_or = translateOrnt(nw_or, next_action);
        }

        if (current_state == TurtleState::FINISHED) {
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
        case MazeDirection::WEST:  pos_.setX(pos_.x() - 1); break;
        case MazeDirection::SOUTH: pos_.setY(pos_.y() + 1); break;
        case MazeDirection::EAST:  pos_.setX(pos_.x() + 1); break;
        case MazeDirection::NORTH: pos_.setY(pos_.y() - 1); break;
    }
    return pos_;
}

int translateOrnt(int orientation, TurtleAction next_action) {
    if (next_action == TurtleAction::TURN_RIGHT) {
        return (orientation + 1) % ORIENTATION_COUNT;
    } else if (next_action == TurtleAction::TURN_LEFT) {
        return (orientation + ORIENTATION_COUNT - 1) % ORIENTATION_COUNT;
    }
    return orientation;
}

bool detectObstacle(QPointF pos_, int facing) {
    QPointF next_pos = translatePos(pos_, facing);
    return bumped(pos_.x(), pos_.y(), next_pos.x(), next_pos.y());
}


