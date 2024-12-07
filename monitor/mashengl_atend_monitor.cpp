#include "monitor_interface.h"

struct AtEndMonitorState {
    bool initialized;
    int current_x;
    int current_y;
    bool reached_end;
};

static AtEndMonitorState state = {false, 0, 0, false};

// Monitor initialization
namespace {
    class MonitorInit {
    public:
        MonitorInit() {
            fprintf(stderr, "I'm running AtEnd Monitor (mashengl) to STDERR\n");
            ROS_WARN("Monitor AtEnd Monitor (mashengl) is running");
        }
    } init;
}

void tickInterrupt(ros::Time t) {
    if (state.reached_end) {
        ROS_WARN("Successful atEnd");
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    state.current_x = x;
    state.current_y = y;
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

    if (x != state.current_x || y != state.current_y) {
        ROS_WARN("VIOLATION: [ATEND] Check at (%d,%d) while turtle is at (%d,%d)", 
                 x, y, state.current_x, state.current_y);
    } else {
        ROS_INFO("[[%ld ns]] Valid atEnd check at (%d,%d)", t.toNSec(), x, y);
        if (atEnd) {
            state.reached_end = true;
        }
    }
}
