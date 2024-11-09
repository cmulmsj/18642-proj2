#include "student_mock.h"

static bool mock_bump = false;
static bool mock_at_end = false;
static int mock_visits = 0;

bool bumped(int x1, int y1, int x2, int y2) {
    return mock_bump;
}

bool atend(int x, int y) {
    return mock_at_end;
}

void displayVisits(int visits) {
    mock_visits = visits;
}

void setMockBump(bool will_bump) {
    mock_bump = will_bump;
}

void setMockAtEnd(bool at_end) {
    mock_at_end = at_end;
}

int getMockVisits() {
    return mock_visits;
}
