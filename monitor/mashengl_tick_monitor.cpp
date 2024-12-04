#include "monitor_interface.h"

// Mailbox structure for incoming interrupts
struct TickMonitorMailbox {
    bool pose_received;
    bool visit_received;
    bool bump_received;
    ros::Time last_time;
};

// Working state
struct TickMonitorState {
    bool tick_active;
    bool pose_seen;
    bool visit_seen;
    bool bump_seen;
};

static TickMonitorMailbox mailbox = {};
static TickMonitorState state = {};
static bool first_run = true;

void tickInterrupt(ros::Time t) {
    if (first_run) {
        first_run = false;
        fprintf(stderr, "I'm running Tick Monitor (mashengl) to STDERR\n");
        ROS_WARN("Monitor Tick Monitor (mashengl) is running");
    }

    if (!state.tick_active) {
        // Start new tick
        state.tick_active = true;
        state.pose_seen = false;
        state.visit_seen = false;
        state.bump_seen = false;
        ROS_INFO("[[%ld ns]] New tick started", t.toNSec());
    } else {
        // End current tick
        state.tick_active = false;
        mailbox = {}; // Clear mailbox
        ROS_INFO("[[%ld ns]] Tick completed", t.toNSec());
    }
    
    mailbox.last_time = t;
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (!state.tick_active) {
        ROS_WARN("VIOLATION: [TICK] Pose interrupt received outside of tick");
        return;
    }
    
    if (state.pose_seen) {
        ROS_WARN("VIOLATION: [TICK] Multiple pose interrupts within single tick");
    }
    state.pose_seen = true;
    ROS_INFO("[[%ld ns]] Pose interrupt received: x=%d, y=%d", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {
    if (!state.tick_active) {
        ROS_WARN("VIOLATION: [TICK] Visit interrupt received outside of tick");
        return;
    }
    
    if (state.visit_seen) {
        ROS_WARN("VIOLATION: [TICK] Multiple visit interrupts within single tick");
    }
    state.visit_seen = true;
    ROS_INFO("[[%ld ns]] Visit interrupt received: visits=%d", t.toNSec(), visits);
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (!state.tick_active) {
        ROS_WARN("VIOLATION: [TICK] Bump interrupt received outside of tick");
        return;
    }
    
    if (state.bump_seen) {
        ROS_WARN("VIOLATION: [TICK] Multiple bump interrupts within single tick");
    }
    state.bump_seen = true;
    ROS_INFO("[[%ld ns]] Bump interrupt received: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    // Not needed for tick monitoring
}
