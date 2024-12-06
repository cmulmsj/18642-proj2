#include "monitor_interface.h"

struct InterruptState {
    bool position;
    bool visit;
    bool wall;
} tick_state = {false, false, false};

namespace {
    class MonitorStartup {
    public:
        MonitorStartup() {
            fprintf(stderr, "Starting Tick Monitor (mashengl)\n");
            ROS_WARN("Monitor Tick Monitor (mashengl) is running");
        }
    } monitor_init;
}

void tickInterrupt(ros::Time t) {
    if (tick_state.position) {
        ROS_WARN("VIOLATION: [TICK] Multiple position updates within tick at %ld", 
                 t.toNSec());
    }
    if (tick_state.visit) {
        ROS_WARN("VIOLATION: [TICK] Multiple visit updates within tick at %ld", 
                 t.toNSec());
    }
    if (tick_state.wall) {
        ROS_WARN("VIOLATION: [TICK] Multiple wall checks within tick at %ld", 
                 t.toNSec());
    }
    
    tick_state = {false, false, false};
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (tick_state.position) {
        ROS_WARN("VIOLATION: [TICK] Multiple position updates within tick at %ld", 
                 t.toNSec());
    }
    tick_state.position = true;
    ROS_INFO("[[%ld ns]] Position update received: x=%d, y=%d", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {
    if (tick_state.visit) {
        ROS_WARN("VIOLATION: [TICK] Multiple visit updates within tick at %ld", 
                 t.toNSec());
    }
    tick_state.visit = true;
    ROS_INFO("[[%ld ns]] Visit update received: visits=%d", t.toNSec(), visits);
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (tick_state.wall) {
        ROS_WARN("VIOLATION: [TICK] Multiple wall checks within tick at %ld", 
                 t.toNSec());
    }
    tick_state.wall = true;
    ROS_INFO("[[%ld ns]] Wall check: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}
