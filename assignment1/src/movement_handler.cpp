#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <assignment1/WaypointMoveAction.h>

typedef actionlib::SimpleActionServer<assignment1::WaypointMoveAction> Server;

class MovementHandler
{
    protected:
    ros::NodeHandle nh_;
    Server as_;  //ActionServer
    std::string action_name_;
    assignment1::WaypointMoveFeedback feedback_;
    assignment1::WaypointMoveResult result_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;


    public:

        MovementHandler(std::string name) :
            as_(nh_, name, boost::bind(&MovementHandler::callbackOnGoal, this, _1), false),
            action_name_(name),
            move_base_client_("move_base", true)
            {
                as_.start();
            }
    
        void callbackOnGoal(const assignment1::WaypointMoveGoal::ConstPtr &goal)
        {
            ROS_INFO("GOAL RECEIVED - MOVING");
            move_base_msgs::MoveBaseGoal  move_goal;
            move_goal.target_pose.header.frame_id = "map";
            move_goal.target_pose.pose.position.x = goal->x;
            move_goal.target_pose.pose.position.y = goal->y;
            move_goal.target_pose.pose.position.z = 0.0;
            move_goal.target_pose.header.stamp = ros::Time::now();

            move_goal.target_pose.pose.orientation.x = 0.0;
            move_goal.target_pose.pose.orientation.y = 0.0;
            move_goal.target_pose.pose.orientation.z = 0.0;
            move_goal.target_pose.pose.orientation.w = 1.0;

            move_base_client_.sendGoal(move_goal);
            bool timeout = move_base_client_.waitForResult(ros::Duration(45.0));
            if(timeout)
            {
                if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    result_.reached=true;
                    ROS_INFO("NAV OK");
                } else
                {
                    result_.reached = false;
                    ROS_ERROR("NAV FAILURE");
                }
            }else
            {
                result_.reached = false;
                ROS_ERROR("NAV TIMEOUT");
            }
            as_.setSucceeded(result_);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "movement_handler");

    MovementHandler move_to_goal("move_to_goal");

    ros::spin();

    return 0;
}

