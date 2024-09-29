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
    static TurtleState current_state = INIT;

    if (count_down == 0) {
        ROS_INFO("MoveTurtle called with position=(%.2f, %.2f), orientation=%d, state=%d",
                 pos_.x(), pos_.y(), nw_or, current_state);

        if (current_state == INIT) {
            addVisit(pos_);
            displayVisits(retrieveVisitCount(pos_));
            ROS_INFO("Initialized: Recorded initial position and displayed visit count.");
        }

        bool has_wall = detectObstacle(pos_, static_cast<Orientation>(nw_or));
        ROS_DEBUG("Obstacle detection result: %s", has_wall ? "True" : "False");

        bool at_goal = atend(static_cast<int>(pos_.x()), static_cast<int>(pos_.y()));
        ROS_DEBUG("At goal detection result: %s", at_goal ? "True" : "False");

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        ROS_INFO("Determined next move: %d", nextMove);

        nw_or = translateOrnt(nw_or, nextMove);
        ROS_INFO("Updated orientation to: %d", nw_or);

        switch (nextMove) {
            case MOVING:
                pos_ = translatePos(pos_, static_cast<Orientation>(nw_or));
                ROS_INFO("Moving to new position: (%.2f, %.2f)", pos_.x(), pos_.y());
                addVisit(pos_);
                displayVisits(retrieveVisitCount(pos_));
                break;
            case TURNING_RIGHT:
                ROS_INFO("Turning right. New orientation: %d", nw_or);
                break;
            case TURNING_LEFT:
                ROS_INFO("Turning left. New orientation: %d", nw_or);
                break;
            case STOPPING:
                ROS_INFO("Stopping movement.");
                break;
            default:
                ROS_ERROR("Received invalid move command: %d", nextMove);
                break;
        }

        if (current_state == GOAL) {
            ROS_INFO("Goal reached. Stopping turtle.");
            return false;
        }

        count_down = MOVE_DELAY;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, Orientation orientation) {
    switch (orientation) {
        case WEST:
            pos_.setX(pos_.x() - 1);
            ROS_DEBUG("Translated position west to (%.2f, %.2f)", pos_.x(), pos_.y());
            break;
        case SOUTH:
            pos_.setY(pos_.y() - 1);
            ROS_DEBUG("Translated position south to (%.2f, %.2f)", pos_.x(), pos_.y());
            break;
        case EAST:
            pos_.setX(pos_.x() + 1);
            ROS_DEBUG("Translated position east to (%.2f, %.2f)", pos_.x(), pos_.y());
            break;
        case NORTH:
            pos_.setY(pos_.y() + 1);
            ROS_DEBUG("Translated position north to (%.2f, %.2f)", pos_.x(), pos_.y());
            break;
        default:
            ROS_ERROR("Invalid Orientation: %d", orientation);
            break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    int new_orientation = orientation;
    switch (nextMove) {
        case TURNING_RIGHT:
            new_orientation = (orientation + 1) % ORIENTATION_COUNT;
            ROS_DEBUG("Orientation changed to (TURNING_RIGHT): %d", new_orientation);
            break;
        case TURNING_LEFT:
            new_orientation = (orientation - 1 + ORIENTATION_COUNT) % ORIENTATION_COUNT;
            ROS_DEBUG("Orientation changed to (TURNING_LEFT): %d", new_orientation);
            break;
        case MOVING:
        case STOPPING:
            ROS_DEBUG("Orientation remains the same: %d", new_orientation);
            break;
        default:
            ROS_ERROR("Invalid Move Command: %d", nextMove);
            break;
    }
    return new_orientation;
}

bool detectObstacle(QPointF pos_, Orientation orient) {
    int start_x = static_cast<int>(pos_.x());
    int start_y = static_cast<int>(pos_.y());
    int end_x = start_x;
    int end_y = start_y;

    switch (orient) {
        case WEST:
            end_y += 1;
            ROS_DEBUG("Orientation WEST: Incremented end_y to %d", end_y);
            break;
        case SOUTH:
            end_x += 1;
            ROS_DEBUG("Orientation SOUTH: Incremented end_x to %d", end_x);
            break;
        case EAST:
            start_x += 1;
            end_x += 1;
            end_y += 1;
            ROS_DEBUG("Orientation EAST: Incremented start_x to %d, end_x to %d, end_y to %d", start_x, end_x, end_y);
            break;
        case NORTH:
            end_x += 1;
            end_y += 1;
            start_y += 1;
            ROS_DEBUG("Orientation NORTH: Incremented end_x to %d, end_y to %d, start_y to %d", end_x, end_y, start_y);
            break;
        default:
            ROS_ERROR("detectObstacle: Invalid Orientation %d", orient);
            return false;
    }

    // Log the coordinates being checked for a wall
    ROS_INFO("detectObstacle: Checking wall between (%d, %d) and (%d, %d)", start_x, start_y, end_x, end_y);

    // Check for a wall between the start and end coordinates
    bool wall_detected = bumped(start_x, start_y, end_x, end_y);

    ROS_INFO("detectObstacle: Wall detected: %s", wall_detected ? "Yes" : "No");
    return wall_detected;
}
