#include "student.h"

bool moveTurtle(QPointF& pos_, int& nw_or) {
    // Call studentTurtleStep to get next movement decision
    bool bumped = false; // will be set by maze code based on wall detection
    turtleMove nextMove = studentTurtleStep(bumped);

    // Translate relative move into absolute coordinates
    pos_ = translatePos(pos_, nextMove);
    nw_or = translateOrnt(nw_or, nextMove);

    // Update the map with the new visit count
    int visits = getVisitCount(pos_.x(), pos_.y());
    displayVisits(visits);

    // Return true if the turtle hasn't reached the end
    return !atend(pos_.x(), pos_.y());
}

QPointF translatePos(QPointF pos_, turtleMove nextMove) {
    // Translate turtle move into absolute coordinates based on move type
    switch (nextMove) {
        case MOVE_FORWARD:
            // Move forward in the current direction
            if (nw_or == 0) pos_.setY(pos_.y() - 1); // north
            if (nw_or == 90) pos_.setX(pos_.x() + 1); // east
            if (nw_or == 180) pos_.setY(pos_.y() + 1); // south
            if (nw_or == 270) pos_.setX(pos_.x() - 1); // west
            break;
        case TURN_LEFT:
        case TURN_RIGHT:
        case NO_MOVE:
        default:
            // No position change for turns or no movement
            break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    // Update the orientation based on the turtle move
    switch (nextMove) {
        case TURN_LEFT:
            orientation = (orientation + 270) % 360;  // Turning left by 90 degrees
            break;
        case TURN_RIGHT:
            orientation = (orientation + 90) % 360;   // Turning right by 90 degrees
            break;
        case MOVE_FORWARD:
        case NO_MOVE:
        default:
            // No change in orientation for forward movement or no movement
            break;
    }
    return orientation;
}
