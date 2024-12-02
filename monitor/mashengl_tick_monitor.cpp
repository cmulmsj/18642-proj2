#include "monitor_interface.h"

// Track interrupt states between ticks
static bool pose_seen = false;
static bool visit_seen = false;
static bool bump_seen = false;
static bool tick_active = false;

void tickInterrupt(ros::Time t) {
    // Check if we had multiple calls of any interrupt type
    if (!tick_active) {
        tick_active = true;
        pose_seen = false;
        visit_seen = false;
        bump_seen = false;
        ROS_INFO("[[%ld ns]] New tick started", t.toNSec());
    } else {
        // Reset for next tick
        tick_active = false;
        ROS_INFO("[[%ld ns]] Tick completed", t.toNSec());
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (!tick_active) {
        ROS_WARN("VIOLATION: Pose interrupt received outside of tick at time %ld", t.toNSec());
        return;
    }
    
    if (pose_seen) {
        ROS_WARN("VIOLATION: Multiple pose interrupts within single tick at time %ld", t.toNSec());
    }
    pose_seen = true;
    ROS_INFO("[[%ld ns]] Pose interrupt received: x=%d, y=%d", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {
    if (!tick_active) {
        ROS_WARN("VIOLATION: Visit interrupt received outside of tick at time %ld", t.toNSec());
        return;
    }
    
    if (visit_seen) {
        ROS_WARN("VIOLATION: Multiple visit interrupts within single tick at time %ld", t.toNSec());
    }
    visit_seen = true;
    ROS_INFO("[[%ld ns]] Visit interrupt received: visits=%d", t.toNSec(), visits);
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (!tick_active) {
        ROS_WARN("VIOLATION: Bump interrupt received outside of tick at time %ld", t.toNSec());
        return;
    }
    
    if (bump_seen) {
        ROS_WARN("VIOLATION: Multiple bump interrupts within single tick at time %ld", t.toNSec());
    }
    bump_seen = true;
    ROS_INFO("[[%ld ns]] Bump interrupt received: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_tick_monitor");
    ROS_WARN("Monitor Tick Monitor (mashengl) is running at %s", ctime(0));
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
