#ifndef TURTLE_STATE_H
#define TURTLE_STATE_H

#include <stdint.h>

// Constants
const int32_t MAZE_SIZE = 100; // Adjusted size to accommodate the turtle's local map
const int32_t START_POS = MAZE_SIZE / 2;

// Enum for directions (relative to turtle's starting orientation)
enum Direction {
    UP = 0,    // Facing up
    RIGHT = 1, // Facing right
    DOWN = 2,  // Facing down
    LEFT = 3   // Facing left
};

// Global variables (extern to share between files)
extern int32_t visitMap[MAZE_SIZE][MAZE_SIZE];
extern int32_t currentX;
extern int32_t currentY;
extern Direction orientation;

// Function prototypes
int32_t getVisitCount(int32_t x, int32_t y);
void setVisitCount(int32_t x, int32_t y, int32_t count);
int getCurrentVisitCount();

#endif // TURTLE_STATE_H
