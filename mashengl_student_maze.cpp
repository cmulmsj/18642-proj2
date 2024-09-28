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

// Function prototypes
bool isFacingWall(QPointF pos_, int nw_or);

// Function to check if the turtle is facing a wall
bool isFacingWall(QPointF pos_, int nw_or) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());

    int x1 = x, y1 = y, x2 = x, y2 = y;

    switch (nw_or) {
        case 0:  // LEFT
            x1 -= 1;
            break;
        case 1:  // UP
            y1 -= 1;
            break;
        case 2:  // RIGHT
            x1 += 1;
            break;
        case 3:  // DOWN
            y1 += 1;
            break;
        default:
            ROS_ERROR("Invalid orientation: %d", nw_or);
            break;
    }

    // The bumped function checks if there's a wall between (x, y) and (x1, y1)
    bool result = bumped(x, y, x1, y1);
    ROS_INFO("isFacingWall: position (%d, %d) facing %d, wall ahead: %d", x, y, nw_or, result);
    return result;
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    // Compute if the turtle is facing a wall
    bool bumpedStatus = isFacingWall(pos_, nw_or);

    // Call the turtle's step function to get the next move
    turtleMove nextMove = studentTurtleStep(bumpedStatus);

    ROS_INFO("moveTurtle: nextMove = %d", nextMove);

    // Update the absolute orientation
    translateOrnt(nw_or, nextMove);

    // Update the absolute position
    translatePos(pos_, nw_or, nextMove);

    // Get the number of visits from the turtle code
    int visits = getCurrentVisitCount();

    // Update the display
    displayVisits(visits);

    // Check if at end
    bool atEnd = atend(static_cast<int>(pos_.x()), static_cast<int>(pos_.y()));
    if (atEnd) {
        ROS_INFO("Turtle has reached the end at position (%f, %f)", pos_.x(), pos_.y());
        return false;  // Stop moving
    }

    return true;  // Continue moving
}

// Update the turtle's absolute position based on the move
void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove) {
    if (nextMove == MOVE_FORWARD) {
        switch (nw_or) {
            case 0: pos_.setX(pos_.x() - 1); break;  // LEFT
            case 1: pos_.setY(pos_.y() - 1); break;  // UP
            case 2: pos_.setX(pos_.x() + 1); break;  // RIGHT
            case 3: pos_.setY(pos_.y() + 1); break;  // DOWN
            default:
                ROS_ERROR("Invalid orientation: %d", nw_or);
                break;
        }
        ROS_INFO("translatePos: Moved to position (%f, %f)", pos_.x(), pos_.y());
    }
    // No position change for turns
}

// Update the turtle's absolute orientation based on the move
void translateOrnt(int& nw_or, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        nw_or = (nw_or + 3) % 4;  // Equivalent to -1 mod 4
        ROS_INFO("translateOrnt: Turned left. New orientation: %d", nw_or);
    } else if (nextMove == TURN_RIGHT) {
        nw_or = (nw_or + 1) % 4;
        ROS_INFO("translateOrnt: Turned right. New orientation: %d", nw_or);
    }
    // No orientation change for MOVE_FORWARD or STOP
}

