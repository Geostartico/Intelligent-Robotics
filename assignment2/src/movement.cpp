#include "movement.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseGoal.h>

// Constructor
Movement::Movement()
    : POS_X_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;  // 0° (Positive X)
          return q;
      }()),
      POS_Y_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 0.707; q.w = 0.707;  // 90° (Positive Y)
          return q;
      }()),
      NEG_X_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 1.0; q.w = 0.0;  // 180° (Negative X)
          return q;
      }()),
      NEG_Y_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = -0.707; q.w = 0.707;  // -90° (Negative Y)
          return q;
      }()),
      ac("move_base", true) {
    // Wait for MoveBase server
    ac.waitForServer();

    // Initialize positions
    std::vector<std::pair<float, float>> docks_offsets = {
        {-DIST, 0}, {0, -DIST}, {DIST, 0}};
    std::vector<std::pair<float, float>> corns_offsets = {
        {-DIST, DIST}, {DIST, DIST}};

    // Set up dock and corner positions
    for (auto off : docks_offsets)
        docks.emplace_back(TABLE_1_X + off.first, TABLE_1_Y - off.second);
    for (auto off : docks_offsets)
        docks.emplace_back(TABLE_2_X - off.first, TABLE_2_Y + off.second);
    for (auto off : corns_offsets)
        corns.emplace_back(TABLE_1_X + off.first, TABLE_1_Y + off.second);
    for (auto off : corns_offsets)
        corns.emplace_back(TABLE_2_X - off.first, TABLE_2_Y - off.second);

    // Move to initial position
    //sendGoalToMoveBase(docks[0].first, 0.0, NEG_Y_ORIENTATION);
    //sendGoalToMoveBase(docks[0].first, docks[0].second, POS_X_ORIENTATION);
    sendGoalToMoveBase(docks[1].first, docks[1].second, NEG_Y_ORIENTATION);
    //cur_pos = 1;
    cur_pos = 2;
}

// Send goal to MoveBase
void Movement::sendGoalToMoveBase(double x, double y, const geometry_msgs::Quaternion& orientation) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = orientation;

    ROS_INFO("Sending goal: x=%.2f, y=%.2f, orientation=[%.2f, %.2f, %.2f, %.2f]",
             x, y, orientation.x, orientation.y, orientation.z, orientation.w);

    ac.sendGoal(goal);
    if (ac.waitForResult(ros::Duration(20))) {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully!");
        } else {
            ROS_WARN("Failed to reach the goal.");
        }
    }
}

// Clockwise movement
void Movement::go_clockwise(int target_pos) {
    geometry_msgs::Quaternion escape_orientation;
    switch (cur_pos) {
        case 2: escape_orientation = POS_X_ORIENTATION; break;
        case 5: escape_orientation = NEG_X_ORIENTATION; break;
        case 3: case 4: escape_orientation = NEG_Y_ORIENTATION; break;
        case 6: case 1: escape_orientation = POS_Y_ORIENTATION; break;
    }
    sendGoalToMoveBase(docks[cur_pos - 1].first, docks[cur_pos - 1].second, escape_orientation);

    if(cur_pos==1 && target_pos>cur_pos) {
        sendGoalToMoveBase(corns[0].first, corns[0].second, POS_Y_ORIENTATION);
        sendGoalToMoveBase(corns[0].first, corns[0].second, POS_X_ORIENTATION);
        cur_pos++;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
        if(target_pos==2){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
            return;
        }
    }
    if(cur_pos==2 && target_pos>cur_pos) {
        sendGoalToMoveBase(corns[1].first, corns[1].second, POS_X_ORIENTATION);
        sendGoalToMoveBase(corns[1].first, corns[1].second, NEG_Y_ORIENTATION);
        cur_pos++;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==3){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(cur_pos==3 && target_pos>cur_pos) {
        cur_pos++;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==4){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(cur_pos==4 && target_pos>cur_pos) {
        sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_Y_ORIENTATION);
        sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_X_ORIENTATION);
        cur_pos++;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
        if(target_pos==5){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
            return;
        }
    }
    if(cur_pos==5 && target_pos>cur_pos) {
        sendGoalToMoveBase(corns[3].first, corns[3].second, NEG_X_ORIENTATION);
        sendGoalToMoveBase(corns[3].first, corns[3].second, POS_Y_ORIENTATION);
        cur_pos++;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        if(target_pos==6){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
            return;
        }
    }
}

// Counter-clockwise movement
void Movement::go_counter_clockwise(int target_pos) {
    geometry_msgs::Quaternion escape_orientation;
    switch (cur_pos) {
        case 2: escape_orientation = NEG_X_ORIENTATION; break;
        case 5: escape_orientation = POS_X_ORIENTATION; break;
        case 3: case 4: escape_orientation = POS_Y_ORIENTATION; break;
        case 6: case 1: escape_orientation = NEG_Y_ORIENTATION; break;
    }
    sendGoalToMoveBase(docks[cur_pos - 1].first, docks[cur_pos - 1].second, escape_orientation);

    if(cur_pos==6 && target_pos<cur_pos) {
        sendGoalToMoveBase(corns[3].first, corns[3].second, NEG_Y_ORIENTATION);
        sendGoalToMoveBase(corns[3].first, corns[3].second, POS_X_ORIENTATION);
        cur_pos--;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
        if(target_pos==5){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
            return;
        }
    }
    if(cur_pos==5 && target_pos<cur_pos) {
        sendGoalToMoveBase(corns[2].first, corns[2].second, POS_X_ORIENTATION);
        sendGoalToMoveBase(corns[2].first, corns[2].second, POS_Y_ORIENTATION);
        cur_pos--;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        if(target_pos==4){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(cur_pos==4 && target_pos<cur_pos) {
        cur_pos--;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        if(target_pos==3){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(cur_pos==3 && target_pos<cur_pos) {
        sendGoalToMoveBase(corns[1].first, corns[1].second, POS_Y_ORIENTATION);
        sendGoalToMoveBase(corns[1].first, corns[1].second, NEG_X_ORIENTATION);
        cur_pos--;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==2){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
            return;
        }
    }
    if(cur_pos==2 && target_pos<cur_pos) {
        sendGoalToMoveBase(corns[0].first, corns[0].second, NEG_X_ORIENTATION);
        sendGoalToMoveBase(corns[0].first, corns[0].second, NEG_Y_ORIENTATION);
        cur_pos--;
        sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==1){
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
            return;
        }
    }
}

// Navigate to target position
void Movement::goAround(int target_pos) {
    if (target_pos > cur_pos)
        go_clockwise(target_pos);
    else if (target_pos < cur_pos)
        go_counter_clockwise(target_pos);
    else
        ROS_WARN("Robot is already in target position.");
}

float Movement::dock_dist(float x, float y, int dock) {
    if(dock<1 || dock>6) {
        ROS_ERROR("Given dock index is not valid.");
        return -1.0;
    }
    float x1 = x;
    float y1 = y;
    float x2 = docks[dock-1].first;
    float y2 = docks[dock-1].second;
    return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}
