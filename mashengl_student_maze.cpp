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
    static bool firstMove = true;
    
    // Calculate the position ahead based on orientation
    QPointF nextPos = translatePos(pos_, FORWARD, nw_or);

    // Check if there's a wall ahead
    bool isBumped = bumped(pos_.x(), pos_.y(), nextPos.x(), nextPos.y());

    ROS_INFO("Maze - Current Pos: (%.0f, %.0f), Orientation: %d, Next Pos: (%.0f, %.0f), Bumped: %d, First Move: %d",
             pos_.x(), pos_.y(), nw_or, nextPos.x(), nextPos.y(), isBumped, firstMove);

    // Call the turtle's decision-making function
    turtleMove nextMove = studentTurtleStep(isBumped);

    if (firstMove) {
        // On the first move, we determine the initial orientation based on where we can move
        if (!isBumped) {
            // We can move in the current direction, so our initial orientation is correct
            ROS_INFO("Initial orientation confirmed: %d", nw_or);
        } else {
            // We hit a wall, so we need to try other directions
            for (int i = 1; i < 4; i++) {
                nw_or = translateOrnt(nw_or, TURN_RIGHT);
                nextPos = translatePos(pos_, FORWARD, nw_or);
                if (!bumped(pos_.x(), pos_.y(), nextPos.x(), nextPos.y())) {
                    ROS_INFO("Initial orientation corrected to: %d", nw_or);
                    break;
                }
            }
        }
        firstMove = false;
    } else {
        // Update the orientation
        nw_or = translateOrnt(nw_or, nextMove);

        // Update the position if moving forward and not bumped
        if (nextMove == FORWARD && !isBumped) {
            pos_ = translatePos(pos_, FORWARD, nw_or);
            ROS_INFO("Maze - Moved to (%.0f, %.0f)", pos_.x(), pos_.y());
        }
    }

    // Check if the turtle has reached the end
    bool atEnd = atend(pos_.x(), pos_.y());

    ROS_INFO("Maze - End of move: Pos: (%.0f, %.0f), Orientation: %d, At End: %d",
             pos_.x(), pos_.y(), nw_or, atEnd);

    // Return true to continue, false to stop the turtle
    return !atEnd;
}


QPointF translatePos(QPointF pos_, turtleMove nextMove, int orientation) {
    switch (nextMove) {
        case FORWARD:
            switch (orientation) {
                case 0: return QPointF(pos_.x() + 1, pos_.y()); // RIGHT
                case 1: return QPointF(pos_.x(), pos_.y() + 1); // DOWN
                case 2: return QPointF(pos_.x() - 1, pos_.y()); // LEFT
                case 3: return QPointF(pos_.x(), pos_.y() - 1); // UP
            }
            break;
        case TURN_LEFT:
        case TURN_RIGHT:
        case STOP:
            return pos_; // No position change for turns or stop
    }
    return pos_; // Default case, should not happen
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

