#include "student.h"
#include <ros/ros.h>
#include <cmath>

// Forward declarations
turtleMove studentTurtleStep(bool bumped, int& newOrientation);
int getVisitCount(int x, int y);

QPointF translatePos(QPointF currentPos, int orientation, turtleMove move) {
    QPointF newPos = currentPos;
    if (move == MOVE) {
        switch (orientation) {
            case 0: newPos.setX(newPos.x() - 1); break; // Left
            case 1: newPos.setY(newPos.y() - 1); break; // Up
            case 2: newPos.setX(newPos.x() + 1); break; // Right
            case 3: newPos.setY(newPos.y() + 1); break; // Down
        }
    }
    ROS_INFO("translatePos: from (%f, %f) to (%f, %f), orientation: %d, move: %d",
             currentPos.x(), currentPos.y(), newPos.x(), newPos.y(), orientation, move);
    return newPos;
}

bool checkBump(QPointF pos, int orientation) {
    QPointF newPos = pos;
    switch (orientation) {
        case 0: newPos.setX(newPos.x() - 1); break; // Left
        case 1: newPos.setY(newPos.y() - 1); break; // Up
        case 2: newPos.setX(newPos.x() + 1); break; // Right
        case 3: newPos.setY(newPos.y() + 1); break; // Down
    }
    return bumped(pos.x(), pos.y(), newPos.x(), newPos.y());
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int timeoutCounter = 0;
    const int TIMEOUT = 40;

    if (timeoutCounter == 0) {
        bool isBumped = checkBump(pos_, nw_or);
        int newOrientation = nw_or;
        turtleMove nextMove = studentTurtleStep(isBumped, newOrientation);
        
        QPointF oldPos = pos_;
        int oldOrientation = nw_or;
        
        QPointF newPos = translatePos(pos_, newOrientation, nextMove);
        
        if (!isBumped) {
            pos_ = newPos;
            nw_or = newOrientation;
            
            int visits = getVisitCount(std::round(pos_.x()), std::round(pos_.y()));
            displayVisits(visits);

            ROS_INFO("Turtle moved from (%f, %f) to (%f, %f), orientation: %d -> %d, visits: %d",
                     oldPos.x(), oldPos.y(), pos_.x(), pos_.y(), oldOrientation, nw_or, visits);
        } else {
            ROS_INFO("Turtle bumped at (%f, %f), orientation: %d", pos_.x(), pos_.y(), nw_or);
        }
        
        timeoutCounter = TIMEOUT;
    } else {
        timeoutCounter--;
    }
    
    return !atend(pos_.x(), pos_.y());
}
