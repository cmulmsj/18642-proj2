#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

const int32_t MAZE_SIZE = 23;
const int32_t START_POS = MAZE_SIZE / 2;

enum Direction { LEFT = 0, UP = 1, RIGHT = 2, DOWN = 3 };

static int32_t currentX = 0;
static int32_t currentY = 0;
static int32_t currentOrientation = RIGHT;  // Start facing right
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};

void updatePosition() {
    switch (currentOrientation) {
        case LEFT:  currentX--; break;
        case UP:    currentY--; break;
        case RIGHT: currentX++; break;
        case DOWN:  currentY++; break;
    }
    ROS_INFO("Relative position updated to (%d, %d)", currentX, currentY);
}

turtleMove studentTurtleStep(bool bumped, int& newOrientation) {
    ROS_INFO("studentTurtleStep called with bumped: %s", bumped ? "true" : "false");

    if (bumped) {
        // If bumped, turn left
        currentOrientation = (currentOrientation - 1 + 4) % 4;
        ROS_INFO("Bumped, turned left, new orientation: %d", currentOrientation);
    } else {
        // If not bumped, try to move forward
        updatePosition();
        ROS_INFO("Moved forward, new position: (%d, %d)", currentX, currentY);
    }

    newOrientation = currentOrientation;
    return MOVE;
}

int getVisitCount(int x, int y) {
    int relX = x - START_POS + currentX;
    int relY = y - START_POS + currentY;
    if (relX >= 0 && relX < MAZE_SIZE && relY >= 0 && relY < MAZE_SIZE) {
        visitMap[relY][relX]++;
        ROS_INFO("Visit count for (%d, %d) updated to %d", x, y, visitMap[relY][relX]);
        return visitMap[relY][relX];
    }
    ROS_WARN("Attempted to access visit count outside of maze bounds: (%d, %d)", x, y);
    return 0;
}
