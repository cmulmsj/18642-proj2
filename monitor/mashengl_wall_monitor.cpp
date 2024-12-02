#include "monitor_interface.h"
#include <queue>
#include <cstdlib>

// Store recent bump checks
static const int MAX_BUMP_MEMORY = 4;
static std::queue<std::pair<Endpoints, bool>> bump_memory;
static Pose last_pose;
static bool first_move = true;

static Endpoints canonicalizeWall(int x1, int y1, int x2, int y2) {
    Endpoints wall;
    if (y1 == y2) {
        if (x1 <= x2) {
            wall = {x1, y1, x2, y2};
        } else {
            wall = {x2, y2, x1, y1};
        }
    } else {
        if (y1 <= y2) {
            wall = {x1, y1, x2, y2};
        } else {
            wall = {x2, y2, x1, y1};
        }
    }
    return wall;
}

static bool wallsEqual(const Endpoints& wall1, const Endpoints& wall2) {
    return wall1.x1 == wall2.x1 && 
           wall1.y1 == wall2.y1 && 
           wall1.x2 == wall2.x2 && 
           wall1.y2 == wall2.y2;
}

static Endpoints wallBetween(const Pose& p1, const Pose& p2) {
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    
    if (std::abs(dx) + std::abs(dy) != 1) {
        ROS_ERROR("Non-adjacent points in wallBetween");
        return {0, 0, 0, 0};
    }

    if (dx == 1) {  // Moving East
        return canonicalizeWall(p1.x+1, p1.y, p1.x+1, p1.y+1);
    } else if (dx == -1) {  // Moving West
        return canonicalizeWall(p1.x, p1.y, p1.x, p1.y+1);
    } else if (dy == 1) {  // Moving South
        return canonicalizeWall(p1.x, p1.y+1, p1.x+1, p1.y+1);
    } else {  // Moving North
        return canonicalizeWall(p1.x, p1.y, p1.x+1, p1.y);
    }
}

static bool bumpedInMemory(const Endpoints& wall) {
    std::queue<std::pair<Endpoints, bool>> temp = bump_memory;
    while (!temp.empty()) {
        auto [stored_wall, was_bumped] = temp.front();
        temp.pop();
        if (wallsEqual(stored_wall, wall)) {
            return was_bumped;
        }
    }
    return true;  // If wall not found in memory, assume blocked for safety
}

void tickInterrupt(ros::Time t) {}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (first_move) {
        last_pose = {x, y};
        first_move = false;
        return;
    }

    // Check if position changed
    if (last_pose.x != x || last_pose.y != y) {
        Pose current_pose = {x, y};
        
        // Get wall between previous and current position
        Endpoints crossed_wall = wallBetween(last_pose, current_pose);
        
        // Check if we tried to cross this wall
        bool wall_blocked = bumpedInMemory(crossed_wall);
        
        if (wall_blocked) {
            ROS_WARN("VIOLATION: Crossed blocked wall from (%d,%d) to (%d,%d) at time %ld", 
                     last_pose.x, last_pose.y, x, y, t.toNSec());
        } else {
            ROS_INFO("[[%ld ns]] Valid movement through checked wall from (%d,%d) to (%d,%d)", 
                     t.toNSec(), last_pose.x, last_pose.y, x, y);
        }
        
        // Clear bump memory after moving
        while (!bump_memory.empty()) {
            bump_memory.pop();
        }
    }
    
    last_pose = {x, y};
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    Endpoints wall = canonicalizeWall(x1, y1, x2, y2);
    
    // Store bump check in circular buffer
    bump_memory.push({wall, bumped});
    if (bump_memory.size() > MAX_BUMP_MEMORY) {
        bump_memory.pop();
    }
    
    ROS_INFO("[[%ld ns]] Bump check: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_wall_monitor");
    ROS_WARN("Monitor Wall Monitor (mashengl) is running at %s", ctime(0));
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
