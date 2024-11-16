/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * 
 * Monitor to check that turtle turns no more than 90 degrees per time step
 */

#include "monitor_interface.h"

// Track last orientation for comparison
static Orientation last_orientation = NORTH;
static bool first_move = true;

// Lookup table for checking valid 90-degree turns
const bool valid_turns[4][4] = {
    //WEST  NORTH EAST  SOUTH (current)
    {true,  true,  false, true}, // WEST (previous)
    {true,  true,  true,  false},// NORTH
    {false, true,  true,  true}, // EAST
    {true,  false, true,  true}  // SOUTH
};

/*
 * Check orientation changes to ensure no more than 90-degree turns
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (first_move) {
        first_move = false;
        last_orientation = o;
        ROS_INFO("[[%ld ns]] Initial orientation: %d", t.toNSec(), o);
        return;
    }

    // Check if turn is valid using lookup table
    if (!valid_turns[last_orientation][o] && last_orientation != o) {
        ROS_WARN("VIOLATION: Invalid turn from %d to %d - turn greater than 90 degrees!", 
                 last_orientation, o);
        ROS_INFO("Time: %ld ns, Position: (%d,%d)", t.toNSec(), x, y);
    }

    // Update last orientation
    if (last_orientation != o) {
        ROS_INFO("[[%ld ns]] Turn detected: %d -> %d", t.toNSec(), last_orientation, o);
        last_orientation = o;
    }
}

// Required empty handlers for other interrupts
void tickInterrupt(ros::Time t) {}
void visitInterrupt(ros::Time t, int visits) {}
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
