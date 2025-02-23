#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>

// typedef for code readability
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Movement {
private:
    // Corridor end and dock distance constants
    static constexpr float MAX_CORRIDOR_X = 6.8;
    static constexpr float DIST = 0.9;

    // Possible robot orientation for movement
    const geometry_msgs::Quaternion POS_X_ORIENTATION;
    const geometry_msgs::Quaternion POS_Y_ORIENTATION;
    const geometry_msgs::Quaternion NEG_X_ORIENTATION;
    const geometry_msgs::Quaternion NEG_Y_ORIENTATION;

    // Attributes to track robot position and save waypoints
    int cur_pos;
    std::vector<std::pair<float, float>> docks;
    std::vector<std::pair<float, float>> corns;
    MoveBaseClient ac;

    void go_clockwise(int target_pos);
    void go_counter_clockwise(int target_pos);
    void sendGoalToMoveBase(double x, double y, const geometry_msgs::Quaternion& orientation);

public:
    // Tables center coordinates
    float TABLE_1_X;
    float TABLE_1_Y;
    float TABLE_2_X;
    float TABLE_2_Y;

    Movement(ros::NodeHandle& nh);
    void detect_tables(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goAround(int target_pos);
    void fix_pos();
    void spin(double yaw);
    float dock_dist(float x, float y, int dock);
    int closest_dock(float x, float y);
};

#endif // MOVEMENT_H
