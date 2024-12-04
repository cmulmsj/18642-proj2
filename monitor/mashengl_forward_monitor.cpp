#include "monitor_interface.h"

struct ForwardMonitorMailbox {
    Pose latest_pose;
    Orientation latest_orientation;
    bool pose_updated;
    bool orientation_updated;
};

struct ForwardMonitorState {
    Pose current_pose;
    Orientation current_orientation;
    bool has_turned;
    bool initialized;
};

static ForwardMonitorMailbox mailbox = {};
static ForwardMonitorState state = {};
static bool first_run = true;

static Orientation getMovementDirection(const Pose& from, const Pose& to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 1) return EAST;
    if (dx == -1) return WEST;
    if (dy == 1) return SOUTH;
    if (dy == -1) return NORTH;
    
    ROS_ERROR("Invalid movement in getMovementDirection");
    return NORTH;
}

void tickInterrupt(ros::Time t) {
    if (first_run) {
        first_run = false;
        fprintf(stderr, "I'm running Forward Monitor (mashengl) to STDERR\n");
        ROS_WARN("Monitor Forward Monitor (mashengl) is running");
    }

    // Reset turn flag at start of tick
    state.has_turned = false;
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (!state.initialized) {
        state.initialized = true;
        state.current_pose = {x, y};
        state.current_orientation = o;
        ROS_INFO("[[%ld ns]] Initial position: (%d,%d), orientation: %d", 
                 t.toNSec(), x, y, o);
        return;
    }

    // Check for rotation
    if (state.current_orientation != o) {
        state.has_turned = true;
        ROS_INFO("[[%ld ns]] Rotation from %d to %d", 
                 t.toNSec(), state.current_orientation, o);
    }

    // Check for movement
    if (state.current_pose.x != x || state.current_pose.y != y) {
        Pose new_pose = {x, y};
        
        // Check if moving in direction facing
        Orientation movement_dir = getMovementDirection(state.current_pose, new_pose);
        if (state.current_orientation != movement_dir) {
            ROS_WARN("VIOLATION: [FORWARD] Movement direction (%d) doesn't match orientation (%d)", 
                     movement_dir, state.current_orientation);
        }
        
        // Check for movement in same tick as rotation
        if (state.has_turned) {
            ROS_WARN("VIOLATION: [FORWARD] Movement occurred in same tick as rotation");
        }
        
        ROS_INFO("[[%ld ns]] Movement from (%d,%d) to (%d,%d), orientation: %d", 
                 t.toNSec(), state.current_pose.x, state.current_pose.y, x, y, o);
    }
    
    state.current_pose = {x, y};
    state.current_orientation = o;
}

void visitInterrupt(ros::Time t, int visits) {}
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}

