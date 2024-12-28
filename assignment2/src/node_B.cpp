#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include "assignment2/object_detect.h"

// CONSTANTS
float TABLE_1_X = 7.82;
float TABLE_1_Y = -1.96;
float TABLE_2_X = 7.82;
float TABLE_2_Y = -3.01;
float DIST      = 2.0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool detection_routine(assignment2::object_detect::Request &req, assignment2::object_detect::Response &res)
{
    if(req.ready == false) {
        ROS_WARN("Detection request is set to false.");
        return false;
    }

    ROS_INFO("Service called. Detection routine starts.");

    MoveBaseClient ac("move_base", true);

    std::vector<std::pair<float, float>> docks_offsets = {
        std::make_pair(-DIST, 0), 
        std::make_pair(0, -DIST), 
        std::make_pair(DIST, 0)
    };
    std::vector<std::pair<float, float>> corns_offsets = {
        std::make_pair(-DIST, -DIST), 
        std::make_pair(DIST, -DIST)
    };

    std::vector<std::pair<float, float>> docks, corns;
    for(auto off : docks_offsets) 
        docks.push_back(std::make_pair(TABLE_2_X + off.first, TABLE_2_Y + off.second));
    for(auto off : corns_offsets) 
        corns.push_back(std::make_pair(TABLE_2_X + off.first, TABLE_2_Y + off.second));

    ROS_INFO("Docking positions:");
    for(auto p : docks)
        ROS_INFO("x=%f y=%f", p.first, p.second);
    ROS_INFO("Corner positions:");
    for(auto p : corns)
        ROS_INFO("x=%f y=%f", p.first, p.second);

    std::vector<std::vector<float>> docks_or = {
        {0.0, 0.0, 0.0, 1.0},
        {0.0, 0.0, 0.707, 0.707},
        {0.0, 0.0, 1.0, 0.0}
    };
    
    // std::vector<std::pair<float, float>> det_path = {docks[0], corns[0], docks[1], corns[1], docks[2]};

    std::vector<float> objs_x, objs_y, docks_x, docks_y;
    std::vector<int> ids;

    for(int i=0; i<docks.size(); i++)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; 
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = docks[i].first;
        goal.target_pose.pose.position.y = docks[i].second;
        goal.target_pose.pose.position.z = 0.0; 
        goal.target_pose.pose.orientation.x = docks_or[i][0];
        goal.target_pose.pose.orientation.y = docks_or[i][1];
        goal.target_pose.pose.orientation.z = docks_or[i][2];
        goal.target_pose.pose.orientation.w = docks_or[i][3];

        ROS_INFO("[%u/%u] Sending goal (docking station).", i+1, docks.size());
        ac.sendGoal(goal);
        ac.waitForResult();

        // DETECTION PHASE HERE

        // Corner visit (to be changed in visit the closest corner between docks)
        if(i < corns.size()) {
            move_base_msgs::MoveBaseGoal goal_corner;
            goal_corner.target_pose.header.frame_id = "map"; 
            goal_corner.target_pose.header.stamp = ros::Time::now();
            goal_corner.target_pose.pose.position.x = corns[i].first;
            goal_corner.target_pose.pose.position.y = corns[i].second;
            goal_corner.target_pose.pose.position.z = 0.0; 
            goal_corner.target_pose.pose.orientation.x = 1.0;
            goal_corner.target_pose.pose.orientation.y = 0.0;
            goal_corner.target_pose.pose.orientation.z = 0.0;
            goal_corner.target_pose.pose.orientation.w = 0.0;

            ROS_INFO("[%u/%u] Sending goal (corner).", i+1, corns.size());
            ac.sendGoal(goal);
            ac.waitForResult();
        }
    }

    res.objs_x = objs_x;
    res.objs_y = objs_y;
    res.docks_x = docks_x;
    res.docks_y = docks_y;
    res.ids = ids;

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_B");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("detection_srv", detection_routine);
    ROS_INFO("Object detection service avaliable.");

    ros::spin();
    return 0;
}
