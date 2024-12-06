/*
 * This monitor checks that between tickInterrupts, there is at most
 * one call to each of poseInterrupt, visitsInterrupt, and bumpInterrupt.
 */
#include "monitor_interface.h"

// Track occurrence of each type of interrupt since last tick
static struct {
    bool pose_seen;
    bool visit_seen;
    bool bump_seen;
} interrupt_flags = {false, false, false};

class MonitorInit {
public:
    MonitorInit() {
        std::cerr << "Monitor ece642rtle_tick_monitor is running" << std::endl;
        ROS_WARN("Monitor ece642rtle_tick_monitor is running");
    }
} init;

void tickInterrupt(ros::Time t) {
    // Check for violations before resetting flags
    if (interrupt_flags.pose_seen) {
        ROS_WARN("VIOLATION: Multiple pose updates in single tick at time %ld", 
                 t.toNSec());
    }
    if (interrupt_flags.visit_seen) {
        ROS_WARN("VIOLATION: Multiple visit updates in single tick at time %ld", 
                 t.toNSec());
    }
    if (interrupt_flags.bump_seen) {
        ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", 
                 t.toNSec());
    }
    
    // Reset flags for next tick
    interrupt_flags.pose_seen = false;
    interrupt_flags.visit_seen = false;
    interrupt_flags.bump_seen = false;
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (interrupt_flags.pose_seen) {
        ROS_WARN("VIOLATION: Multiple pose updates in single tick at time %ld", 
                 t.toNSec());
    }
    interrupt_flags.pose_seen = true;
}

void visitInterrupt(ros::Time t, int visits) {
    if (interrupt_flags.visit_seen) {
        ROS_WARN("VIOLATION: Multiple visit updates in single tick at time %ld", 
                 t.toNSec());
    }
    interrupt_flags.visit_seen = true;
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (interrupt_flags.bump_seen) {
        ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", 
            t.toNSec()); 
    } 
    interrupt_flags.bump_seen = true; 
} 

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}




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
