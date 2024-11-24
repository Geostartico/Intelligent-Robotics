#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_move_test");
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBase("move_base", true);
    moveBase.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.seq =1;
    goal.target_pose.pose.position.x = 11.7;
    goal.target_pose.pose.position.y = -3.3;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send goal.
    moveBase.sendGoal(goal);
    moveBase.waitForResult();

    bool positionReached = true;

    if(moveBase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("YAY");
    } else {
        ROS_INFO("NAY");
    }
}
