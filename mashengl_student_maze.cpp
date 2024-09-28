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
#include <ros/ros.h>

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    // Calculate the position ahead based on current orientation
    QPointF nextPos = translatePos(pos_, FORWARD, nw_or);

    // Check if there's a wall ahead
    bool isBumped = bumped(pos_.x(), pos_.y(), nextPos.x(), nextPos.y());

    ROS_INFO("Maze - Current Pos: (%.0f, %.0f), Orientation: %d, Next Pos: (%.0f, %.0f), Bumped: %d",
             pos_.x(), pos_.y(), nw_or, nextPos.x(), nextPos.y(), isBumped);

    // Call the turtle's decision-making function
    turtleMove nextMove = studentTurtleStep(isBumped);

    // Update the orientation
    nw_or = translateOrnt(nw_or, nextMove);

    // Update the position if moving forward and not bumped
    if (nextMove == FORWARD && !isBumped) {
        pos_ = nextPos;
        ROS_INFO("Maze - Moved to (%.0f, %.0f)", pos_.x(), pos_.y());
    } else if (nextMove == FORWARD && isBumped) {
        ROS_WARN("Maze - Bumped into wall at (%.0f, %.0f)", nextPos.x(), nextPos.y());
    }

    ROS_INFO("Maze - End of move: Pos: (%.0f, %.0f), Orientation: %d",
             pos_.x(), pos_.y(), nw_or);

    // Check if the turtle has reached the end
    bool atEnd = atend(pos_.x(), pos_.y());

    // Return true to continue, false to stop the turtle
    return !atEnd;
}


QPointF translatePos(QPointF pos_, turtleMove nextMove, int orientation) {
    if (nextMove == FORWARD) {
        switch (orientation) {
            case 0: return QPointF(pos_.x() + 1, pos_.y()); // RIGHT
            case 1: return QPointF(pos_.x(), pos_.y() + 1); // DOWN
            case 2: return QPointF(pos_.x() - 1, pos_.y()); // LEFT
            case 3: return QPointF(pos_.x(), pos_.y() - 1); // UP
        }
    }
    // For turns and stop, position doesn't change
    return pos_;
}


int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case TURN_LEFT:
            return (orientation + 3) % 4;
        case TURN_RIGHT:
            return (orientation + 1) % 4;
        case FORWARD:
        case STOP:
            return orientation; // No orientation change for forward or stop
    }
    return orientation; // Default case, should not happen
}


