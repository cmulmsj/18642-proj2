#include "monitor_interface.h"

// Track interrupt counts between ticks
struct InterruptState {
    bool pose_seen;
    bool visit_seen;
    bool bump_seen;
    bool in_tick;
} static state = {false, false, false, false};

// Monitor initialization
namespace {
    class MonitorInit {
    public:
        MonitorInit() {
            fprintf(stderr, "I'm running Tick Monitor (mashengl) to STDERR\n");
            ROS_WARN("Monitor Tick Monitor (mashengl) is running");
        }
    } init;
}

void tickInterrupt(ros::Time t) {
    ROS_INFO("[[%ld ns]] New tick started", t.toNSec());
    
    // Reset state for new tick
    state.pose_seen = false;
    state.visit_seen = false;
    state.bump_seen = false;
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (state.pose_seen) {
        ROS_WARN("VIOLATION: Multiple pose updates in single tick at time %ld", 
                 t.toNSec());
        return;
    }
    state.pose_seen = true;
    ROS_INFO("[[%ld ns]] Pose update received: (%d,%d)", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {
    if (state.visit_seen) {
        ROS_WARN("VIOLATION: Multiple visit updates in single tick at time %ld", 
                 t.toNSec());
        return;
    }
    state.visit_seen = true;
    ROS_INFO("[[%ld ns]] Visit count update: %d", t.toNSec(), visits);
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (state.bump_seen) {
        ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", 
                 t.toNSec());
        return;
    }
    state.bump_seen = true;
    ROS_INFO("[[%ld ns]] Bump check: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    // Not needed for tick monitoring
}




// #include "monitor_interface.h"
// // Track occurrence of each type of interrupt since last tick
// static struct {
//     bool pose_seen;
//     bool visit_seen;
//     bool bump_seen;
// } interrupt_flags = {false, false, false};
// class MonitorInit {
// public:
//     MonitorInit() {
//         std::cerr << "Monitor ece642rtle_tick_monitor is running" << std::endl;
//         ROS_WARN("Monitor ece642rtle_tick_monitor is running");
//     }
// } init;
// void tickInterrupt(ros::Time t) {
//     // Check for violations before resetting flags
//     if (interrupt_flags.pose_seen) {
//         ROS_WARN("VIOLATION: Multiple pose updates in single tick at time %ld", 
//                  t.toNSec());
//     }
//     if (interrupt_flags.visit_seen) {
//         ROS_WARN("VIOLATION: Multiple visit updates in single tick at time %ld", 
//                  t.toNSec());
//     }
//     if (interrupt_flags.bump_seen) {
//         ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", 
//                  t.toNSec());
//     }

//     // Reset flags for next tick
//     interrupt_flags.pose_seen = false;
//     interrupt_flags.visit_seen = false;
//     interrupt_flags.bump_seen = false;
// }
// void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
//     if (interrupt_flags.pose_seen) {
//         ROS_WARN("VIOLATION: Multiple pose updates in single tick at time %ld", 
//                  t.toNSec());
//     }
//     interrupt_flags.pose_seen = true;
// }
// void visitInterrupt(ros::Time t, int visits) {
//     if (interrupt_flags.visit_seen) {
//         ROS_WARN("VIOLATION: Multiple visit updates in single tick at time %ld", 
//                  t.toNSec());
//     }
//     interrupt_flags.visit_seen = true;
// }
// void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
//     if (interrupt_flags.bump_seen) {
//         ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", t.toNSec()); } interrupt_flags.bump_seen = true; } 

// void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
