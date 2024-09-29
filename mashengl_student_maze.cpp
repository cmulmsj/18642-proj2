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
    static int count_down = 0;
    static State current_state = INIT;

    if (count_down == 0) {
        if (current_state == INIT) {
            addVisit(pos_);
            displayVisits(getVisit(pos_));
            ROS_INFO("moveTurtle: Initial position recorded and visits displayed.");
        }

        bool has_wall = detectObstacle(pos_, static_cast<Orientation>(nw_or));
        bool at_goal = atend(pos_.x(), pos_.y());

        ROS_INFO("moveTurtle: Checking obstacles - Has Wall: %d, At Goal: %d", has_wall, at_goal);

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("moveTurtle: Orientation updated to %d after move %d", nw_or, nextMove);

        if (nextMove == MOVE) {
            if (!has_wall) {
                pos_ = translatePos(pos_, static_cast<Orientation>(nw_or));
                addVisit(pos_);
                displayVisits(getVisit(pos_));
                ROS_INFO("moveTurtle: Moved to new position (%.0f, %.0f)", pos_.x(), pos_.y());
            } else {
                ROS_WARN("moveTurtle: Attempted to move forward but a wall was detected. Position unchanged.");
            }
        }

        if (current_state == GOAL) {
            ROS_INFO("moveTurtle: Turtle has reached the goal. Halting.");
            return false;
        }

        count_down = TIMEOUT;
        ROS_INFO("moveTurtle: Move completed. Countdown reset to %d.", count_down);
        return true;
    }

    count_down--;
    ROS_DEBUG("moveTurtle: Countdown decreased to %d.", count_down);
    return false;
}

QPointF translatePos(QPointF pos_, Orientation orientation) {
    switch (orientation) {
        case LEFT:
            pos_.setX(pos_.x() - 1);
            ROS_INFO("translatePos: Moving LEFT to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case DOWN:
            pos_.setY(pos_.y() - 1); // Assuming y decreases when moving south
            ROS_INFO("translatePos: Moving DOWN to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case RIGHT:
            pos_.setX(pos_.x() + 1);
            ROS_INFO("translatePos: Moving RIGHT to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case UP:
            pos_.setY(pos_.y() + 1); // Assuming y increases when moving north
            ROS_INFO("translatePos: Moving UP to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        default:
            ROS_ERROR("translatePos: Invalid Orientation %d", orientation);
            break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    int new_orientation = orientation;
    switch (nextMove) {
        case TURNRIGHT:
            new_orientation = (orientation + 1) % NUM_ORIENTATIONS;
            ROS_INFO("translateOrnt: TURNRIGHT. New Orientation: %d", new_orientation);
            break;
        case TURNLEFT:
            new_orientation = (orientation - 1 + NUM_ORIENTATIONS) % NUM_ORIENTATIONS;
            ROS_INFO("translateOrnt: TURNLEFT. New Orientation: %d", new_orientation);
            break;
        case MOVE:
        case STOP:
            ROS_INFO("translateOrnt: %s. Orientation remains at %d", 
                     (nextMove == MOVE) ? "MOVE" : "STOP", new_orientation);
            break;
        default:
            ROS_ERROR("translateOrnt: Invalid Move %d. Orientation remains at %d", nextMove, new_orientation);
            break;
    }
    return new_orientation;
}

bool detectObstacle(QPointF current_pos, CompassDirection direction) {
    int x1 = static_cast<int>(current_pos.x());
    int y1 = static_cast<int>(current_pos.y());
    int x2 = x1, y2 = y1;

    switch (direction) {
        case CompassDirection::WEST:
            x2 = x1 - 1;
            break;
        case CompassDirection::SOUTH:
            y2 = y1 - 1;
            break;
        case CompassDirection::EAST:
            x2 = x1 + 1;
            break;
        case CompassDirection::NORTH:
            y2 = y1 + 1;
            break;
        default:
            ROS_ERROR("Invalid Compass Direction in detectObstacle.");
            return false;
    }

    ROS_DEBUG("Checking for obstacle between (%d, %d) and (%d, %d)", x1, y1, x2, y2);
    bool is_bumped = bumped(x1, y1, x2, y2);
    ROS_INFO("Obstacle detected ahead: %s", is_bumped ? "Yes" : "No");
    return is_bumped;
}



