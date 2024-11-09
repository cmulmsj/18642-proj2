#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <stdint.h>

enum turtleAction {FORWARD, LEFT, RIGHT};

typedef struct {
    uint8_t x;
    uint8_t y;
} coordinate;

typedef struct {
    turtleAction action;
    bool validAction;
    uint8_t visitCount;
} turtleMove;

bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);

void setMockBump(bool will_bump);
void setMockAtEnd(bool at_end);
int getMockVisits();

#endif
