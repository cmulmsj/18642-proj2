#ifndef MASHENGL_MAZE_PARAMS_H
#define MASHENGL_MAZE_PARAMS_H

const int GRID_SIZE = 23;
const int MOVE_DELAY = 40;
const int DIRECTION_COUNT = 4;

// Updated enum classes to match CompassDirection and TurtleAction
enum class TurtleDirection : int {
    WEST = 0,
    SOUTH = 1,
    EAST = 2,
    NORTH = 3
};

enum class TurtleCommand : int {
    ADVANCE,
    ROTATE_RIGHT,
    ROTATE_LEFT,
    HALT
};

enum class NavigationMode : int {
    INITIAL,
    MOVING,
    ADJUSTING,
    GOAL_REACHED
};

#endif // MASHENGL_MAZE_PARAMS_H
