#include "monitor_interface.h"

struct TurnMonitorMailbox {
    Orientation latest_orientation;
    bool orientation_updated;
};

struct TurnMonitorState {
    Orientation current_orientation;
    bool initialized;
};

static TurnMonitorMailbox mailbox = {};
static TurnMonitorState state = {};

// Monitor initialization
namespace {
    class MonitorInit {
    public:
        MonitorInit() {
            fprintf(stderr, "I'm running Turn Monitor (mashengl) to STDERR\n");
            ROS_WARN("Monitor Turn Monitor (mashengl) is running");
        }
    } init;
}

// Lookup table for valid 90-degree turns
static const bool valid_turns[4][4] = {
    //WEST  NORTH EAST  SOUTH (current)
    {true,  true,  false, true}, // WEST (previous)
    {true,  true,  true,  false},// NORTH
    {false, true,  true,  true}, // EAST
    {true,  false, true,  true}  // SOUTH
};

void tickInterrupt(ros::Time t) {
    // Update state from mailbox
    if (mailbox.orientation_updated) {
        Orientation new_orientation = mailbox.latest_orientation;
        
        if (state.initialized) {
            if (state.current_orientation != new_orientation) {
                if (!valid_turns[state.current_orientation][new_orientation]) {
                    ROS_WARN("VIOLATION: [TURN] Invalid turn from %d to %d - turn greater than 90 degrees!", 
                             state.current_orientation, new_orientation);
                } else {
                    ROS_INFO("[[%ld ns]] Valid turn from %d to %d", 
                             t.toNSec(), state.current_orientation, new_orientation);
                }
            }
        }
        
        state.current_orientation = new_orientation;
        mailbox.orientation_updated = false;
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (!state.initialized) {
        state.initialized = true;
        state.current_orientation = o;
        ROS_INFO("[[%ld ns]] Initial orientation: %d", t.toNSec(), o);
        return;
    }

    mailbox.latest_orientation = o;
    mailbox.orientation_updated = true;
}

void visitInterrupt(ros::Time t, int visits) {}
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
