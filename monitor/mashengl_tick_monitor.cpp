// #include "monitor_interface.h"

// struct InterruptState {
//     bool position;
//     bool visit;
//     bool wall;
// } tick_state = {false, false, false};

// namespace {
//     class MonitorStartup {
//     public:
//         MonitorStartup() {
//             fprintf(stderr, "Starting Tick Monitor (mashengl)\n");
//             ROS_WARN("Monitor Tick Monitor (mashengl) is running");
//         }
//     } monitor_init;
// }

// void tickInterrupt(ros::Time t) {
//     if (tick_state.position) {
//         ROS_WARN("VIOLATION: [TICK] Multiple position updates within tick at %ld", 
//                  t.toNSec());
//     }
//     if (tick_state.visit) {
//         ROS_WARN("VIOLATION: [TICK] Multiple visit updates within tick at %ld", 
//                  t.toNSec());
//     }
//     if (tick_state.wall) {
//         ROS_WARN("VIOLATION: [TICK] Multiple wall checks within tick at %ld", 
//                  t.toNSec());
//     }
    
//     tick_state = {false, false, false};
// }

// void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
//     if (tick_state.position) {
//         ROS_WARN("VIOLATION: [TICK] Multiple position updates within tick at %ld", 
//                  t.toNSec());
//     }
//     tick_state.position = true;
//     ROS_INFO("[[%ld ns]] Position update received: x=%d, y=%d", t.toNSec(), x, y);
// }

// void visitInterrupt(ros::Time t, int visits) {
//     if (tick_state.visit) {
//         ROS_WARN("VIOLATION: [TICK] Multiple visit updates within tick at %ld", 
//                  t.toNSec());
//     }
//     tick_state.visit = true;
//     ROS_INFO("[[%ld ns]] Visit update received: visits=%d", t.toNSec(), visits);
// }

// void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
//     if (tick_state.wall) {
//         ROS_WARN("VIOLATION: [TICK] Multiple wall checks within tick at %ld", 
//                  t.toNSec());
//     }
//     tick_state.wall = true;
//     ROS_INFO("[[%ld ns]] Wall check: (%d,%d)->(%d,%d) = %s", 
//              t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
// }

// void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}



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
        ROS_WARN("VIOLATION: Multiple bump checks in single tick at time %ld", t.toNSec()); } interrupt_flags.bump_seen = true; } 

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
