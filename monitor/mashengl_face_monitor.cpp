// #include "monitor_interface.h"

// // Monitor state
// static Pose current_pose;
// static Orientation current_orientation;
// static bool pose_initialized = false;

// static Endpoints getExpectedWall(int x, int y, Orientation o) {
//     Endpoints wall;
//     switch(o) {
//         case WEST:
//             wall = {x, y, x, y+1};
//             break;
//         case NORTH:
//             wall = {x, y, x+1, y};
//             break;
//         case EAST:
//             wall = {x+1, y, x+1, y+1};
//             break;
//         case SOUTH:
//             wall = {x, y+1, x+1, y+1};
//             break;
//         default:
//             ROS_ERROR("Invalid orientation in getExpectedWall");
//             wall = {0, 0, 0, 0};
//     }
//     return wall;
// }

// static bool wallsEqual(const Endpoints& w1, const Endpoints& w2) {
//     return (w1.x1 == w2.x1 && w1.y1 == w2.y1 && w1.x2 == w2.x2 && w1.y2 == w2.y2) ||
//            (w1.x1 == w2.x2 && w1.y1 == w2.y2 && w1.x2 == w2.x1 && w1.y2 == w2.y1);
// }

// void tickInterrupt(ros::Time t) {}

// void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
//     current_pose = {x, y};
//     current_orientation = o;
//     pose_initialized = true;
//     ROS_INFO("[[%ld ns]] Updated position: (%d,%d), orientation: %d", 
//              t.toNSec(), x, y, o);
// }

// void visitInterrupt(ros::Time t, int visits) {}

// void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
//     if (!pose_initialized) {
//         ROS_WARN("VIOLATION: Bump check before position initialization at time %ld", t.toNSec());
//         return;
//     }

//     Endpoints expected = getExpectedWall(current_pose.x, current_pose.y, current_orientation);
//     Endpoints checked = {x1, y1, x2, y2};
    
//     if (!wallsEqual(expected, checked)) {
//         ROS_WARN("VIOLATION: Checking wall (%d,%d)->(%d,%d) while facing %d at (%d,%d) at time %ld", 
//                  x1, y1, x2, y2, current_orientation, current_pose.x, current_pose.y, t.toNSec());
//     } else {
//         ROS_INFO("[[%ld ns]] Valid bump check for wall (%d,%d)->(%d,%d)", 
//                  t.toNSec(), x1, y1, x2, y2);
//     }
// }

// void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}


#include "monitor_interface.h"

struct FaceMonitorMailbox {
    Pose latest_pose;
    Orientation latest_orientation;
    bool pose_updated;
    bool orientation_updated;
};

struct FaceMonitorState {
    Pose current_pose;
    Orientation current_orientation;
    bool initialized;
};

static FaceMonitorMailbox mailbox = {};
static FaceMonitorState state = {};
static bool first_run = true;

static Endpoints getExpectedWall(int x, int y, Orientation o) {
    Endpoints wall;
    switch(o) {
        case WEST:
            wall = {x, y, x, y+1};
            break;
        case NORTH:
            wall = {x, y, x+1, y};
            break;
        case EAST:
            wall = {x+1, y, x+1, y+1};
            break;
        case SOUTH:
            wall = {x, y+1, x+1, y+1};
            break;
        default:
            ROS_ERROR("Invalid orientation in getExpectedWall");
            wall = {0, 0, 0, 0};
    }
    return wall;
}

static bool wallsEqual(const Endpoints& w1, const Endpoints& w2) {
    return (w1.x1 == w2.x1 && w1.y1 == w2.y1 && w1.x2 == w2.x2 && w1.y2 == w2.y2) ||
           (w1.x1 == w2.x2 && w1.y1 == w2.y2 && w1.x2 == w2.x1 && w1.y2 == w2.y1);
}

void tickInterrupt(ros::Time t) {
    if (first_run) {
        first_run = false;
        fprintf(stderr, "I'm running Face Monitor (mashengl) to STDERR\n");
        ROS_WARN("Monitor Face Monitor (mashengl) is running");
    }
    
    // Update state from mailbox
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
    
    if (!state.initialized) {
        state.initialized = true;
        state.current_pose = {x, y};
        state.current_orientation = o;
    }
    
    ROS_INFO("[[%ld ns]] Updated position: (%d,%d), orientation: %d", 
             t.toNSec(), x, y, o);
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (!state.initialized) {
        ROS_WARN("VIOLATION: [FACE] Bump check before position initialization");
        return;
    }

    Endpoints expected = getExpectedWall(state.current_pose.x, state.current_pose.y, 
                                       state.current_orientation);
    Endpoints checked = {x1, y1, x2, y2};
    
    if (!wallsEqual(expected, checked)) {
        ROS_WARN("VIOLATION: [FACE] Checking wall (%d,%d)->(%d,%d) while facing %d at (%d,%d)", 
                 x1, y1, x2, y2, state.current_orientation, 
                 state.current_pose.x, state.current_pose.y);
    } else {
        ROS_INFO("[[%ld ns]] Valid bump check for wall (%d,%d)->(%d,%d)", 
                 t.toNSec(), x1, y1, x2, y2);
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}


