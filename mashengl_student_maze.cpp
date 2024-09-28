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

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    static bool firstMove = true;
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    // Calculate the position ahead based on orientation
    switch (nw_or) {
        case 0: x2 = x1 + 1; break; // RIGHT
        case 1: y2 = y1 + 1; break; // DOWN
        case 2: x2 = x1 - 1; break; // LEFT
        case 3: y2 = y1 - 1; break; // UP
    }

    // Check if there's a wall ahead
    bool isBumped = bumped(x1, y1, x2, y2);

    ROS_INFO("Maze - Current Pos: (%d, %d), Orientation: %d, Next Pos: (%d, %d), Bumped: %d, First Move: %d",
             x1, y1, nw_or, x2, y2, isBumped, firstMove);

    // Call the turtle's decision-making function
    turtleMove nextMove = studentTurtleStep(isBumped);

    if (firstMove) {
        // On the first move, we determine the initial orientation based on where we can move
        if (!isBumped) {
            // We can move in the current direction, so our initial orientation is correct
            ROS_INFO("Initial orientation confirmed: %d", nw_or);
        } else {
            // We hit a wall, so we need to try other directions
            for (int i = 1; i < 4; i++) {
                nw_or = (nw_or + 1) % 4;
                switch (nw_or) {
                    case 0: x2 = x1 + 1; y2 = y1; break; // RIGHT
                    case 1: x2 = x1; y2 = y1 + 1; break; // DOWN
                    case 2: x2 = x1 - 1; y2 = y1; break; // LEFT
                    case 3: x2 = x1; y2 = y1 - 1; break; // UP
                }
                if (!bumped(x1, y1, x2, y2)) {
                    ROS_INFO("Initial orientation corrected to: %d", nw_or);
                    break;
                }
            }
        }
        firstMove = false;
    } else {
        // Update the orientation
        if (nextMove == TURN_LEFT) {
            nw_or = (nw_or + 3) % 4;
        } else if (nextMove == TURN_RIGHT) {
            nw_or = (nw_or + 1) % 4;
        } else if (nextMove == FORWARD && !isBumped) {
            // Only update position if moving forward and not bumped
            pos_.setX(x2);
            pos_.setY(y2);
            ROS_INFO("Maze - Moved to (%d, %d)", x2, y2);
        }
    }

    // Check if the turtle has reached the end
    bool atEnd = atend(pos_.x(), pos_.y());

    ROS_INFO("Maze - End of move: Pos: (%d, %d), Orientation: %d, At End: %d",
             (int)pos_.x(), (int)pos_.y(), nw_or, atEnd);

    // Return true to continue, false to stop the turtle
    return !atEnd;
}


int translateOrnt(int orientation, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        orientation = (orientation + 3) % 4; // Turn left
    } else if (nextMove == TURN_RIGHT) {
        orientation = (orientation + 1) % 4; // Turn right
    }
    // No orientation change for FORWARD or STOP
    return orientation;
}

