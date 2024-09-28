#include "student.h"
#include <ros/ros.h>

// Forward declarations
turtleMove studentTurtleStep(bool bumped);
int getVisitCount(int x, int y);

// Function to translate relative position to absolute position
QPointF translatePos(QPointF currentPos, int currentOrientation, turtleMove move) {
    QPointF newPos = currentPos;
    if (move == MOVE) {
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
        case MOVE: return (currentOrientation + 1) % 4; // Assuming MOVE also means turn right
        default:   return currentOrientation;
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
        
        int visits = getVisitCount(pos_.x(), pos_.y());
        displayVisits(visits);
        
        return !atend(pos_.x(), pos_.y());
    }
    
    return true;
}
