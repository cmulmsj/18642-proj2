#include "student.h"
#include "mashengl_turtle_state.h"
#include <ros/ros.h>
#include <QPointF>

bool isFacingWall(QPointF pos_, int nw_or) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    int x1 = x, y1 = y;

    switch (nw_or) {
        case 0: y1 -= 1; break; // Up
        case 1: x1 += 1; break; // Right
        case 2: y1 += 1; break; // Down
        case 3: x1 -= 1; break; // Left
    }

    return bumped(x, y, x1, y1);
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    static bool firstCall = true;
    if (firstCall) {
        nw_or = 3; // Facing left
        orientation = LEFT;
        firstCall = false;
        ROS_INFO("Initial orientation: %d", nw_or);
    }

    // Check if the turtle is facing a wall
    bool bumpedStatus = isFacingWall(pos_, nw_or);
    ROS_INFO("Current position: (%.2f, %.2f), Orientation: %d, Facing wall: %s", 
             pos_.x(), pos_.y(), nw_or, bumpedStatus ? "true" : "false");

    // Get the next move from the turtle
    turtleMove nextMove = studentTurtleStep(bumpedStatus);
    ROS_INFO("Next move: %d", static_cast<int>(nextMove));

    // Update orientation
    int oldOrnt = nw_or;
    translateOrnt(nw_or, nextMove);
    ROS_INFO("Orientation changed from %d to %d", oldOrnt, nw_or);

    // Update position only if the move is MOVE_FORWARD and not bumped
    if (nextMove == MOVE_FORWARD && !bumpedStatus) {
        QPointF oldPos = pos_;
        translatePos(pos_, nw_or, nextMove);
        ROS_INFO("Position changed from (%.2f, %.2f) to (%.2f, %.2f)", 
                 oldPos.x(), oldPos.y(), pos_.x(), pos_.y());
    } else if (nextMove == MOVE_FORWARD && bumpedStatus) {
        ROS_WARN("Attempted to move into a wall. Move ignored.");
    }

    // Get the number of visits from the turtle code
    int visits = getCurrentVisitCount();
    ROS_INFO("Current visit count: %d", visits);

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

void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove) {
    if (nextMove == MOVE_FORWARD) {
        QPointF newPos = pos_;
        switch (nw_or) {
            case 0: newPos.setY(newPos.y() - 1); break; // Up
            case 1: newPos.setX(newPos.x() + 1); break; // Right
            case 2: newPos.setY(newPos.y() + 1); break; // Down
            case 3: newPos.setX(newPos.x() - 1); break; // Left
        }
        
        // Check if the new position is valid (not hitting a wall)
        if (!bumped(pos_.x(), pos_.y(), newPos.x(), newPos.y())) {
            pos_ = newPos;
            ROS_INFO("Position updated to (%.2f, %.2f)", pos_.x(), pos_.y());
        } else {
            ROS_WARN("Move blocked by wall. Position remains (%.2f, %.2f)", pos_.x(), pos_.y());
        }
    }
}

void translateOrnt(int& nw_or, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        nw_or = (nw_or + 3) % 4; // Equivalent to -1 mod 4
    } else if (nextMove == TURN_RIGHT) {
        nw_or = (nw_or + 1) % 4;
    }
    // No orientation change for MOVE_FORWARD or STOP

    // Update the turtle's internal orientation
    orientation = static_cast<Direction>(nw_or);
    ROS_INFO("New orientation: %d", nw_or);
}
