/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: [Your Last Update Date]
 *
 * This file contains the turtle's movement logic using the right-hand rule.
 * The turtle operates based on its local perception without knowledge of
 * absolute positions or orientations.
 */

#include "student.h"

// Local variables to keep track of the turtle's state
static int visitMap[100][100]; // Turtle's relative visit map (can adjust size)
static int currentX = 50, currentY = 50; // Starting position in the relative grid
static int timeoutCounter = 0;
static const int TIMEOUT = 10;

bool studentMoveTurtle(QPointF& pos_, int32_t& nw_or) {
    static bool firstCall = true;
    if (firstCall) {
        incrementVisitCount(currentX, currentY);
        firstCall = false;
    }

    // Handle turtle movement or stopping based on bump sensor
    if (timeoutCounter == 0) {
        bool bumped = bumped(pos_.x(), pos_.y(), currentX, currentY);

        // Get the next move based on bump sensor and internal turtle logic
        turtleMove nextMove = studentTurtleStep(bumped);

        if (nextMove == MOVE_FORWARD) {
            // Move in the direction the turtle is facing
            switch (nw_or) {
                case 0: currentY--; break; // North
                case 90: currentX++; break; // East
                case 180: currentY++; break; // South
                case 270: currentX--; break; // West
            }
            incrementVisitCount(currentX, currentY);
        } else if (nextMove == TURN_LEFT || nextMove == TURN_RIGHT) {
            // Only adjust orientation for turns
            nw_or = translateOrnt(nw_or, nextMove);
        }

        // Reset the timeout counter for the next tick
        timeoutCounter = TIMEOUT;
    } else {
        // Decrement timeout counter
        timeoutCounter--;
    }

    // Return true if the turtle should continue, false if it has reached the end
    return !atend(currentX, currentY);
}

turtleMove studentTurtleStep(bool bumped) {
    // Basic logic to avoid obstacles (can be expanded)
    if (bumped) {
        return TURN_LEFT; // Turn left if bumped
    } else {
        return MOVE_FORWARD; // Move forward otherwise
    }
}

// Increments the visit count for the turtle's current position
void incrementVisitCount(int x, int y) {
    visitMap[x][y]++;
}

// Returns the visit count for a given position
int getVisitCount(int x, int y) {
    return visitMap[x][y];
}

