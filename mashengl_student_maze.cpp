#include "student.h"
#include "mashengl_turtle_state.h"
#include <ros/ros.h>
#include <QPointF>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>

/*
 * This file translates the turtle's relative moves into absolute positions
 * and handles interactions with the maze environment.
 * It keeps track of the absolute coordinates and orientation of the turtle.
 */

// Function prototypes
bool isFacingWall(QPointF pos_, int nw_or);

// Function to check if the turtle is facing a wall
bool isFacingWall(QPointF pos_, int nw_or) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());

    int x1 = x;
    int y1 = y;

    // Calculate the position in front of the turtle based on its orientation
    switch (nw_or) {
        case 0: y1 -= 1; break; // Up
        case 1: x1 += 1; break; // Right
        case 2: y1 += 1; break; // Down
        case 3: x1 -= 1; break; // Left
        default:
            ROS_ERROR("Invalid orientation in isFacingWall");
            break;
    }

    // Use the bumped function to check for walls
    return bumped(x, y, x1, y1);
}

// In mashengl_student_maze.cpp

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    static bool firstCall = true;
    if (firstCall) {
        nw_or = 3; // Facing left
        firstCall = false;
    }

    // Check if the turtle is facing a wall
    bool bumpedStatus = isFacingWall(pos_, nw_or);

    // Get the next move from the turtle
    turtleMove nextMove = studentTurtleStep(bumpedStatus);

    // Update orientation
    translateOrnt(nw_or, nextMove);

    // Update position only if the move is MOVE_FORWARD and not bumped
    if (nextMove == MOVE_FORWARD && !bumpedStatus) {
        translatePos(pos_, nw_or, nextMove);
    }

    // Get the number of visits from the turtle code
    int visits = getCurrentVisitCount();

    // Update the display
    displayVisits(visits);

    // Check if at end
    bool atEnd = atend(static_cast<int>(pos_.x()), static_cast<int>(pos_.y()));
    if (atEnd) {
        ROS_INFO("Turtle has reached the end of the maze.");
        return false; // Stop the turtle
    }

    return true; // Continue moving the turtle
}
// Update the turtle's absolute position based on the move
void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove) {
    if (nextMove == MOVE_FORWARD) {
        switch (nw_or) {
            case 0: pos_.setY(pos_.y() - 1); break; // Up
            case 1: pos_.setX(pos_.x() + 1); break; // Right
            case 2: pos_.setY(pos_.y() + 1); break; // Down
            case 3: pos_.setX(pos_.x() - 1); break; // Left
            default:
                ROS_ERROR("Invalid orientation in translatePos");
                break;
        }
    }
    // No position change for turns
}

// In mashengl_student_maze.cpp

void translateOrnt(int& nw_or, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        nw_or = (nw_or + 3) % 4; // Equivalent to -1 mod 4
    } else if (nextMove == TURN_RIGHT) {
        nw_or = (nw_or + 1) % 4;
    }
    // No orientation change for MOVE_FORWARD or STOP

    // Update the turtle's internal orientation
    orientation = static_cast<Direction>(nw_or);
}

