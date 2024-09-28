#include "student.h"

// Define the TurtleMove enum if not already defined
enum TurtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    NO_MOVE
};

// Function to translate relative position to absolute position
QPointF translatePos(QPointF currentPos, int currentOrientation, TurtleMove move) {
    QPointF newPos = currentPos;
    switch (move) {
        case MOVE_FORWARD:
            switch (currentOrientation) {
                case 0: newPos.setX(newPos.x() - 1); break; // Left
                case 1: newPos.setY(newPos.y() - 1); break; // Up
                case 2: newPos.setX(newPos.x() + 1); break; // Right
                case 3: newPos.setY(newPos.y() + 1); break; // Down
            }
            break;
        default:
            break; // No position change for turns or no move
    }
    return newPos;
}

// Function to translate orientation based on the move
int translateOrnt(int currentOrientation, TurtleMove move) {
    switch (move) {
        case TURN_LEFT:
            return (currentOrientation - 1 + 4) % 4;
        case TURN_RIGHT:
            return (currentOrientation + 1) % 4;
        default:
            return currentOrientation;
    }
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    bool bumped = isBumped(pos_, nw_or); // You need to implement this function
    TurtleMove nextMove = studentTurtleStep(bumped);
    
    QPointF newPos = translatePos(pos_, nw_or, nextMove);
    int newOrientation = translateOrnt(nw_or, nextMove);
    
    // Check if the new position is valid (not bumping into a wall)
    if (!isBumped(newPos, newOrientation)) {
        pos_ = newPos;
        nw_or = newOrientation;
        
        // Update visit count and display
        int visits = updateVisitCount(pos_); // You need to implement this function
        displayVisits(visits);
        
        return true;
    }
    
    return false;
}

// You need to implement these helper functions
bool isBumped(QPointF pos, int orientation) {
    // Check if the turtle would bump into a wall at the given position and orientation
    // Return true if it would bump, false otherwise
}

int updateVisitCount(QPointF pos) {
    // Update and return the visit count for the given position
    // This should interact with the turtle's local map in mashengl_student_turtle.cpp
}
