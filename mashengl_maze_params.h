#ifndef MAZE_PARAMS_H
#define MAZE_PARAMS_H

const int GRID_DIMENSION = 23;
const int MOVE_COOLDOWN = 40;
const int ORIENTATION_COUNT = 4;
const int FULL_ROTATION = 4;

enum class MazeDirection : int {
    WESTWARD = 0,
    SOUTHWARD = 1,
    EASTWARD = 2,
    NORTHWARD = 3
};

enum class TurtleAction : int {
    PROCEED,
    PIVOT_CLOCKWISE,
    PIVOT_COUNTERCLOCKWISE,
    HALT
};

enum class NavigationPhase : int {
    SCOUTING,
    REORIENTING,
    RETREATING,
    MISSION_COMPLETE
};

#endif // MAZE_PARAMS_H
