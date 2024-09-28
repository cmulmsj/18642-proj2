#include "student.h"
#include <ros/ros.h>

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
        case 0: // Left
            x1 -= 1;
            break;
        case 1: // Up
            y1 -= 1;
            break;
        case 2: // Right
            x1 += 1;
            break;
        case 3: // Down
            y1 += 1;
            break;
        default:
            ROS_ERROR("Invalid orientation in isFacingWall");
            break;
    }

    // Use the bumped function to check for walls
    return bumped(x, y, x1, y1);
}

// Main function to move the turtle
bool moveTurtle(QPointF& pos_, int& nw_or)
{
    // Determine if the turtle is facing a wall
    bool bumpedStatus = isFacingWall(pos_, nw_or);

    // Get the next move from the turtle
    turtleMove nextMove = studentTurtleStep(bumpedStatus);

    // Update orientation
    translateOrnt(nw_or, nextMove);

    // Update position
    translatePos(pos_, nw_or, nextMove);

    // Get the number of visits from the turtle code
    int visits = getCurrentVisitCount();

    // Update the display
    displayVisits(visits);

    // Check if at end
    bool atEnd = atend(static_cast<int>(pos_.x()), static_cast<int>(pos_.y()));
    if (atEnd) {
        ROS_INFO("Turtle has reached the end of the maze.");
        return false;
    }

    return true;
}

// Update the turtle's absolute position based on the move
void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove) {
    if (nextMove == MOVE_FORWARD) {
        switch (nw_or) {
            case 0: pos_.setX(pos_.x() - 1); break; // Left
            case 1: pos_.setY(pos_.y() - 1); break; // Up
            case 2: pos_.setX(pos_.x() + 1); break; // Right
            case 3: pos_.setY(pos_.y() + 1); break; // Down
            default:
                ROS_ERROR("Invalid orientation in translatePos");
                break;
        }
    }
    // No position change for turns
}

// Update the turtle's absolute orientation based on the move
void translateOrnt(int& nw_or, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        nw_or = (nw_or + 3) % 4; // Equivalent to -1 mod 4
    } else if (nextMove == TURN_RIGHT) {
        nw_or = (nw_or + 1) % 4;
    }
    // No orientation change for MOVE_FORWARD or STOP
}
