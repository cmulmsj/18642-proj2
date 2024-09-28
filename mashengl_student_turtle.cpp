/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

// Turtle's orientation (relative)
enum Direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };
static Direction orientation = LEFT; // Turtle starts facing LEFT

/**
 * Determines the turtle's next move based on whether it has bumped into a wall.
 */
turtleMove studentTurtleStep(bool bumped) {
    static int state = 0; // 0: Try to turn right, 1: Move forward, 2: Turn left

    // Debugging statement
    ROS_INFO("State: %d, Bumped: %d", state, bumped);

    turtleMove nextMove;

    if (state == 0) {
        // Try to turn right
        nextMove = TURN_RIGHT;
        state = 1;
    } else if (state == 1) {
        // After turning right, attempt to move forward
        if (bumped) {
            // Can't move forward, need to turn left
            nextMove = TURN_LEFT;
            state = 2;
        } else {
            // Move forward
            nextMove = FORWARD;
            state = 0;
        }
    } else if (state == 2) {
        // After turning left, try to move forward
        if (bumped) {
            // Can't move forward, need to turn left again
            nextMove = TURN_LEFT;
            // Stay in state 2 to keep turning left until we can move forward
        } else {
            // Move forward
            nextMove = FORWARD;
            state = 0;
        }
    }

    return nextMove;
}
