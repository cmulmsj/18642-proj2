#include "monitor_interface.h"

struct AtEndMonitorMailbox {
    Pose latest_pose;
    bool pose_updated;
    bool maze_completed;
};

struct AtEndMonitorState {
    Pose current_pose;
    bool initialized;
    bool maze_completed;
};

static AtEndMonitorMailbox mailbox = {};
static AtEndMonitorState state = {};
static bool first_run = true;

void tickInterrupt(ros::Time t) {
    if (first_run) {
        first_run = false;
        fprintf(stderr, "I'm running AtEnd Monitor (mashengl) to STDERR\n");
        ROS_WARN("Monitor AtEnd Monitor (mashengl) is running");
    }

    // Process mailbox updates
    if (mailbox.pose_updated) {
        state.current_pose = mailbox.latest_pose;
        mailbox.pose_updated = false;
    }
    
    // Output success message to flush warnings if maze is completed
    if (state.maze_completed) {
        ROS_WARN("Successful atEnd");
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    mailbox.latest_pose = {x, y};
    mailbox.pose_updated = true;
    state.initialized = true;
    
    ROS_INFO("[[%ld ns]] Position updated: (%d,%d)", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (!state.initialized) {
        ROS_WARN("VIOLATION: [ATEND] Check before position initialization");
        return;
    }

    if (x != state.current_pose.x || y != state.current_pose.y) {
        ROS_WARN("VIOLATION: [ATEND] Check at (%d,%d) while turtle is at (%d,%d)", 
                 x, y, state.current_pose.x, state.current_pose.y);
    } else {
        ROS_INFO("[[%ld ns]] Valid atEnd check at (%d,%d)", t.toNSec(), x, y);
    }

    if (atEnd) {
        state.maze_completed = true;
        mailbox.maze_completed = true;
    }
}


