#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Movement {
private:
    static constexpr float TABLE_1_X = 7.82;
    static constexpr float TABLE_1_Y = -1.96;
    static constexpr float TABLE_2_X = 7.82;
    static constexpr float TABLE_2_Y = -3.01;
    static constexpr float DIST = 1.0;

    const geometry_msgs::Quaternion POS_X_ORIENTATION;
    const geometry_msgs::Quaternion POS_Y_ORIENTATION;
    const geometry_msgs::Quaternion NEG_X_ORIENTATION;
    const geometry_msgs::Quaternion NEG_Y_ORIENTATION;

    int cur_pos;
    std::vector<std::pair<float, float>> docks;
    std::vector<std::pair<float, float>> corns;
    MoveBaseClient ac;

    void go_clockwise(int target_pos);
    void go_counter_clockwise(int target_pos);
    void sendGoalToMoveBase(double x, double y, const geometry_msgs::Quaternion& orientation);

public:
    Movement();
    void goAround(int target_pos);
    float dock_dist(float x, float y, int dock);
    int closest_dock(float x, float y);
};

#endif // MOVEMENT_H
