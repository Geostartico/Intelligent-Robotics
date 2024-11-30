#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>

void lookDown() {
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> client("/head_controller/point_head_action", true);

    ROS_INFO("Waiting for the action server to start...");
    client.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");

    control_msgs::PointHeadGoal goal;

    geometry_msgs::PointStamped target_point;
    target_point.header.frame_id = "base_link"; 
    target_point.header.stamp = ros::Time::now();
    target_point.point.x = 1.0; 
    target_point.point.y = 0.0; 
    target_point.point.z = -0.5;

    goal.target = target_point;

    goal.pointing_frame = "head_2_link";
    goal.pointing_axis.x = 1.0;      
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 0.0;

    goal.min_duration = ros::Duration(0.5); 
    goal.max_velocity = 1.0;               

    client.sendGoal(goal);

    bool finished_before_timeout = client.waitForResult(ros::Duration(20.0));

    if (finished_before_timeout) {
        ROS_INFO("Head movement succeeded.");
    } else {
        ROS_WARN("Head movement did not finish before the timeout.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "look_down_node");
    ros::NodeHandle nh;

    lookDown();

    return 0;
}
