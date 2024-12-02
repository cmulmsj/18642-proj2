#include "monitor_interface.h"

static Orientation last_orientation;
static bool first_move = true;

// Lookup table for valid 90-degree turns
static const bool valid_turns[4][4] = {
    //WEST  NORTH EAST  SOUTH (current)
    {true,  true,  false, true}, // WEST (previous)
    {true,  true,  true,  false},// NORTH
    {false, true,  true,  true}, // EAST
    {true,  false, true,  true}  // SOUTH
};

void tickInterrupt(ros::Time t) {}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (first_move) {
        first_move = false;
        last_orientation = o;
        ROS_INFO("[[%ld ns]] Initial orientation: %d", t.toNSec(), o);
        return;
    }

    // Check if orientation changed
    if (last_orientation != o) {
        if (!valid_turns[last_orientation][o]) {
            ROS_WARN("VIOLATION: Invalid turn from %d to %d - turn greater than 90 degrees at time %ld!", 
                     last_orientation, o, t.toNSec());
        } else {
            ROS_INFO("[[%ld ns]] Valid turn from %d to %d", t.toNSec(), last_orientation, o);
        }
    }
    
    last_orientation = o;
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
