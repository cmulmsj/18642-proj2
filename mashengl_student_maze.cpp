#include "student.h"
#include <ros/ros.h>

// Enum for turtle moves
enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    NO_MOVE
};

// Function to translate relative position to absolute position
QPointF translatePos(QPointF currentPos, int currentOrientation, turtleMove move) {
    QPointF newPos = currentPos;
    if (move == MOVE_FORWARD) {
        switch (currentOrientation) {
            case 0: newPos.setX(newPos.x() - 1); break; // Left
            case 1: newPos.setY(newPos.y() - 1); break; // Up
            case 2: newPos.setX(newPos.x() + 1); break; // Right
            case 3: newPos.setY(newPos.y() + 1); break; // Down
        }
    }
    return newPos;
}

// Function to translate orientation based on the move
int translateOrnt(int currentOrientation, turtleMove move) {
    switch (move) {
        case TURN_LEFT:  return (currentOrientation - 1 + 4) % 4;
        case TURN_RIGHT: return (currentOrientation + 1) % 4;
        default:         return currentOrientation;
    }
}

// Function to check if the turtle would bump into a wall
bool checkBump(QPointF pos, int orientation) {
    int futureX = pos.x(), futureY = pos.y();
    int futureX2 = pos.x(), futureY2 = pos.y();

    if (orientation < 2) {  
        if (orientation == 0) futureY2 += 1;
        else                  futureX2 += 1;
    } else {  
        futureX2 += 1; futureY2 += 1;
        if (orientation == 2) futureX += 1;
        else                  futureY += 1;
    }

    return bumped(futureX, futureY, futureX2, futureY2);
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    bool isBumped = checkBump(pos_, nw_or);
    turtleMove nextMove = studentTurtleStep(isBumped);
    
    QPointF newPos = translatePos(pos_, nw_or, nextMove);
    int newOrientation = translateOrnt(nw_or, nextMove);
    
    if (!checkBump(newPos, newOrientation)) {
        pos_ = newPos;
        nw_or = newOrientation;
        
        int visits = updateVisitCount(pos_.x(), pos_.y());
        displayVisits(visits);
        
        return !atend(pos_.x(), pos_.y());
    }
    
    return true;
}

// Declare functions that will be implemented in mashengl_student_turtle.cpp
turtleMove studentTurtleStep(bool bumped);
int updateVisitCount(int x, int y);
