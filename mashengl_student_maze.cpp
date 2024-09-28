#include "student.h"
#include <ros/ros.h>

// Forward declarations
turtleMove studentTurtleStep(bool bumped);
int getVisitCount(int x, int y);

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
    ROS_INFO("translatePos: from (%f, %f) to (%f, %f), orientation: %d, move: %d",
             currentPos.x(), currentPos.y(), newPos.x(), newPos.y(), currentOrientation, move);
    return newPos;
}

int translateOrnt(int currentOrientation, turtleMove move) {
    int newOrientation = currentOrientation;
    if (move == MOVE) {
        newOrientation = (currentOrientation + 1) % 4; // Assuming MOVE also means turn right
    }
    ROS_INFO("translateOrnt: from %d to %d, move: %d", currentOrientation, newOrientation, move);
    return newOrientation;
}

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
    static int timeoutCounter = 0;
    const int TIMEOUT = 40;

    if (timeoutCounter == 0) {
        bool isBumped = checkBump(pos_, nw_or);
        turtleMove nextMove = studentTurtleStep(isBumped);
        
        QPointF oldPos = pos_;
        int oldOrientation = nw_or;
        
        pos_ = translatePos(pos_, nw_or, nextMove);
        nw_or = translateOrnt(nw_or, nextMove);
        
        if (!checkBump(pos_, nw_or)) {
            int visits = getVisitCount(pos_.x(), pos_.y());
            displayVisits(visits);

            ROS_INFO("Turtle moved from (%f, %f) to (%f, %f), orientation: %d -> %d",
                     oldPos.x(), oldPos.y(), pos_.x(), pos_.y(), oldOrientation, nw_or);
        } else {
            pos_ = oldPos;
            nw_or = oldOrientation;
            ROS_INFO("Turtle bumped at (%f, %f), orientation: %d", pos_.x(), pos_.y(), nw_or);
        }
        
        timeoutCounter = TIMEOUT;
    } else {
        timeoutCounter--;
    }
    
    return !atend(pos_.x(), pos_.y());
}
