/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
 */

#include "student.h"

bool need_second_turn = false;

// Visit tracking array
static int visit_number[23][23] = {0};
TurtleState state = INIT;

// Function: record_path
// Purpose:
//   Records the turtle's path in the maze by incrementing the visit count
//   for the current position.
// Inputs:
//   - pos_: Current position of the turtle.
void record_path(QPointF &pos_)
{
    const int TURTLE_START = 11;
    int x = static_cast<int>(pos_.x()) + TURTLE_START;
    int y = static_cast<int>(pos_.y()) + TURTLE_START;
    visit_number[x][y] += 1;
}

// Function: getVisitNumber
// Purpose:
//   Gets the visit count for a specific position in the maze.
// Inputs:
//   - pos_: Current position of the turtle.
int getVisitNumber(Point &pos_)
{
    const int TURTLE_START = 11;
    Point turtle_location;
    turtle_location.x = pos_.x + TURTLE_START;
    turtle_location.y = pos_.y + TURTLE_START;
    return visit_number[turtle_location.x][turtle_location.y];
}

// Function: studentTurtleStep
// Purpose:
//   Determines the turtle's next move based on its current state, orientation, the least-visited direction,
//   and the presence of any obstacles (bumps). Handles necessary turns, including 180-degree turns,
//   to align the turtle towards the least-visited direction.
// Inputs:
//   - at_end: Boolean indicating if the turtle has reached the goal position.
//   - least_visited_direction: Integer representing the direction with the least visits (0-3), or -1 if no valid direction.
//   - bump: Boolean indicating if there's an obstacle directly ahead in the current orientation.
//   - orientation: Integer representing the turtle's current orientation (0-3).
// Outputs:
//   - Returns a turtleMove enum value indicating the turtle's next action (MOVE, TURNLEFT, TURNRIGHT, STOP).
turtleMove studentTurtleStep(bool at_end, int32_t least_visited_direction, bool bump, int32_t orientation)
{
    turtleMove nextMove = STOP;

    if (at_end)
    {
        state = AT_GOAL;
    }

    if (least_visited_direction == -1)
    {
        state = AT_GOAL;
    }

    int direction_gap = (least_visited_direction - orientation + 4) % 4;

    switch (state)
    {
    case INIT:
        nextMove = MOVE;
        state = DECIDE;
        break;

    case DECIDE:
        if (need_second_turn)
        {
            nextMove = TURNLEFT;
            need_second_turn = false;
        }
        else
        {
            switch (direction_gap)
            {
            case FORWARD:
                if (bump)
                {
                    nextMove = STOP;
                    break;
                }
                else
                {
                    nextMove = MOVE;
                    break;
                }
            case TURN_RIGHT:
                nextMove = TURNRIGHT;
                state = TURNING_RIGHT;
                break;
            case TURN_LEFT:
                nextMove = TURNLEFT;
                state = TURNING_LEFT;
                break;
            case TURN_AROUND:
                nextMove = TURNLEFT;
                need_second_turn = true;
                state = TURNING_AROUND;
                break;
            default:
                ROS_ERROR("Invalid direction gap calculation");
                nextMove = STOP;
                state = AT_GOAL;
                break;
            }
        }
        break;
    case TURNING_LEFT:
        nextMove = STOP;
        state = DECIDE;
        break;
    case TURNING_RIGHT:
        nextMove = STOP;
        state = DECIDE;
        break;
    case TURNING_AROUND:
        if (need_second_turn)
        {
            nextMove = TURNLEFT;
            need_second_turn = false;
        }
        else
        {
            nextMove = STOP;
            state = DECIDE;
        }
        break;
    case AT_GOAL:
        nextMove = STOP;
        break;
    default:
        ROS_ERROR("Invalid state");
        nextMove = STOP;
        state = AT_GOAL;
        break;
    }

    return nextMove;
}

