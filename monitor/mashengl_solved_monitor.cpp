#include "monitor_interface.h"

struct SolvedMonitorMailbox {
    Pose latest_pose;
    Orientation latest_orientation;
    bool at_end;
    bool pose_updated;
    bool orientation_updated;
};

struct SolvedMonitorState {
    Pose current_pose;
    Orientation current_orientation;
    bool maze_completed;
};

static SolvedMonitorMailbox mailbox = {};
static SolvedMonitorState state = {};
static bool first_run = true;

void tickInterrupt(ros::Time t) {
    if (first_run) {
        first_run = false;
        fprintf(stderr, "I'm running Solved Monitor (mashengl) to STDERR\n");
        ROS_WARN("Monitor Solved Monitor (mashengl) is running");
    }

    if (mailbox.pose_updated) {
        state.current_pose = mailbox.latest_pose;
        mailbox.pose_updated = false;
    }
    if (mailbox.orientation_updated) {
        state.current_orientation = mailbox.latest_orientation;
        mailbox.orientation_updated = false;
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    mailbox.latest_pose = {x, y};
    mailbox.latest_orientation = o;
    mailbox.pose_updated = true;
    mailbox.orientation_updated = true;

    if (state.maze_completed) {
        if (x != state.current_pose.x || y != state.current_pose.y) {
            ROS_WARN("VIOLATION: [SOLVED] Movement after maze completion");
        }
        if (o != state.current_orientation) {
            ROS_WARN("VIOLATION: [SOLVED] Rotation after maze completion");
        }
    }
    
    ROS_INFO("[[%ld ns]] Position update: (%d,%d), orientation: %d", 
             t.toNSec(), x, y, o);
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (state.maze_completed) {
        ROS_WARN("VIOLATION: [SOLVED] Bump check after maze completion");
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (atEnd) {
        state.maze_completed = true;
        ROS_INFO("[[%ld ns]] Maze completed at position (%d,%d)", 
                 t.toNSec(), x, y);
    }
}

