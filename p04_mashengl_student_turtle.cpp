/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule.
 *
 */

#include "student.h"
#include <stdint.h>
#include <ros/ros.h>
#include <array>

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

// Constants
const int32_t MOVE_DELAY = 40;
const int32_t GRID_SIZE = 23;
const int32_t INIT_POS = GRID_SIZE / 2;

// Enums for stronger typing and switch statement usage
enum class Heading : int8_t
{
    West,
    North,
    East,
    South
};
enum class Action : int8_t
{
    Advance,
    RotateClockwise,
    RotateCounterClockwise
};

// Typedef for geometric pairs
typedef struct
{
    int32_t x;
    int32_t y;
} Coordinate;

// Struct to encapsulate turtle data
struct TurtleData
{
    Coordinate position;
    Heading facing;
    Action nextAction;
    int32_t delayCounter;
    bool reachedGoal;
};

// Global variables (minimized)
static std::array<std::array<int32_t, GRID_SIZE>, GRID_SIZE> cellVisits = {0};
static TurtleData turtle = {{INIT_POS, INIT_POS}, Heading::North, Action::Advance, 0, false};

// Const array for movement offsets
const std::array<Coordinate, 4> MOVE_OFFSETS = {{{-1, 0}, {0, -1}, {1, 0}, {0, 1}}};

/**
 * Updates the turtle's state based on its current action and whether it hit an obstacle.
 *
 * @param hitObstacle Boolean indicating if the turtle hit an obstacle.
 */
void updateTurtleState(bool hitObstacle)
{
    switch (turtle.nextAction)
    {
    case Action::Advance:
        turtle.nextAction = Action::RotateClockwise;
        break;
    case Action::RotateClockwise:
    case Action::RotateCounterclockwise:
        if (hitObstacle)
        {
            turtle.nextAction = Action::RotateCounterclockwise;
            turtle.facing = static_cast<Heading>((static_cast<int8_t>(turtle.facing) + 3) % 4);
        }
        else
        {
            turtle.nextAction = Action::Advance;
        }
        break;
    default:
        ROS_ERROR("Unexpected turtle action: %d", static_cast<int8_t>(turtle.nextAction));
        turtle.nextAction = Action::Advance;
        break;
    }
    if (turtle.nextAction == Action::RotateClockwise)
    {
        turtle.facing = static_cast<Heading>((static_cast<int8_t>(turtle.facing) + 1) % 4);
    }
}

/**
 * Checks if the turtle will collide with an obstacle in its next move.
 *
 * @param pos Current position of the turtle.
 * @return Boolean indicating if a collision will occur.
 */
bool checkCollision(const QPointF &pos)
{
    int32_t checkX = static_cast<int32_t>(pos.x()) + MOVE_OFFSETS[static_cast<int8_t>(turtle.facing)].x;
    int32_t checkY = static_cast<int32_t>(pos.y()) + MOVE_OFFSETS[static_cast<int8_t>(turtle.facing)].y;
    return bumped(checkX, checkY,
                  checkX + (turtle.facing == Heading::East ? 1 : 0),
                  checkY + (turtle.facing == Heading::South ? 1 : 0));
}

/**
 * Moves the turtle to its next position and updates the visit count.
 *
 * @param pos Reference to the turtle's position to be updated.
 */
void moveTurtle(QPointF &pos)
{
    Coordinate offset = MOVE_OFFSETS[static_cast<int8_t>(turtle.facing)];
    pos.setX(pos.x() + offset.x);
    pos.setY(pos.y() + offset.y);
    turtle.position.x += offset.x;
    turtle.position.y += offset.y;

    if (turtle.position.x >= 0 && turtle.position.x < GRID_SIZE &&
        turtle.position.y >= 0 && turtle.position.y < GRID_SIZE)
    {
        cellVisits[turtle.position.y][turtle.position.x]++;
        displayVisits(cellVisits[turtle.position.y][turtle.position.x]);
    }
    else
    {
        ROS_ERROR("Turtle position out of bounds: (%d, %d)", turtle.position.x, turtle.position.y);
    }
}

/**
 * Main function to move the turtle through the maze.
 *
 * @param pos Reference to the turtle's position.
 * @param orientation Reference to the turtle's orientation.
 * @return Boolean indicating if the turtle should continue moving.
 */
bool studentMoveTurtle(QPointF &pos, int32_t &orientation)
{
    if (turtle.delayCounter == 0)
    {
        bool hitObstacle = checkCollision(pos);
        turtle.reachedGoal = atend(pos.x(), pos.y());

        updateTurtleState(hitObstacle);

        if (turtle.nextAction == Action::Advance && !turtle.reachedGoal)
        {
            moveTurtle(pos);
        }

        orientation = static_cast<int32_t>(turtle.facing);
        turtle.delayCounter = MOVE_DELAY;
    }
    else
    {
        turtle.delayCounter--;
    }

    return !turtle.reachedGoal;
}