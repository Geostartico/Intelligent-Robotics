#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include "assignment2/object_detect.h"
#include "assignment2/ObjectMoveAction.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include "assignment2/apriltag_detect.h"


void moveArmRoutine(){
    
    actionlib::SimpleActionClient<assignment2::ObjectMoveAction> ac("move_object", true);
    ROS_INFO("Waiting for Movemente_Handler server to start...");
    ac.waitForServer();
            
    assignment2::ObjectMoveGoal goal_pick;
    goal_pick.pick = true;
    goal_pick.tgt_id = 1;
    goal_pick.tgt_pose.position.x =  0.3;
    goal_pick.tgt_pose.position.y =  0.2;
    goal_pick.tgt_pose.position.z =  0.4;

    tf2::Quaternion tgt_yaw_qt;
    tgt_yaw_qt.setRPY(0,0,M_PI/4);

    goal_pick.tgt_pose.orientation.w = tgt_yaw_qt.w();
    goal_pick.tgt_pose.orientation.x = tgt_yaw_qt.x();
    goal_pick.tgt_pose.orientation.y = tgt_yaw_qt.y();
    goal_pick.tgt_pose.orientation.z = tgt_yaw_qt.z();

         
    ac.sendGoal(goal_pick);
    ac.waitForResult(ros::Duration(30.0));
    ROS_INFO("Movemente_Handler server started. Now looking for the closest waypoint.");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_C_tester");
    ros::NodeHandle n;

    moveArmRoutine();

    ros::spin();
    return 0;
}