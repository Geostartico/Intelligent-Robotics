#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <assignment1/WaypointMoveAction.h>

typedef actionlib::SimpleActionServer<assignment1::WaypointMoveAction> Server;

struct blackROI{
    double x_lower;
    double x_max;
    double y_lower;
    double y_max;
};

struct  robotPos{
    double x_robot;
    double y_robot;
};

class MovementHandler
{
    protected:
    ros::NodeHandle nh_;
    Server as;  //ActionServer
    std::string action_name;
    assignment1::WaypointMoveFeedback feedback;
    assignment1::WaypointMoveResult result;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

    public:

        MovementHandler(std::string name) :
            as(nh_, name, boost::bind(&MovementHandler::callbackOnGoal, this, _1), false),
            action_name(name),
            move_base_client("move_base", true)
            {
                as.start();
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

            move_base_client.sendGoal(move_goal);

            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(20);

            blackROI  fuckin_table{6.78863,7.78863,-1.82448,-2.82448};
            
            move_base_msgs::MoveBaseGoal  move_emergency_goal;
            move_emergency_goal.target_pose.header.frame_id = "map";
            move_emergency_goal.target_pose.pose.position.x = fuckin_table.x_max;
            move_emergency_goal.target_pose.pose.position.y = fuckin_table.y_max;
            move_emergency_goal.target_pose.pose.position.z = 0.0;
            move_emergency_goal.target_pose.header.stamp = ros::Time::now();

            move_emergency_goal.target_pose.pose.orientation.x = 0.0;
            move_emergency_goal.target_pose.pose.orientation.y = 0.0;
            move_emergency_goal.target_pose.pose.orientation.z = 0.0;
            move_emergency_goal.target_pose.pose.orientation.w = 1.0;
            
            ros::Rate rate(10.0);
            while (ros::ok())
            {
                robotPos currentPos = currentRobotPos();
                
                if ((ros::Time::now() - start_time) >  timeout)
                {
                    ROS_ERROR("Navigation timed out - Cancelling goal...");
                    move_base_client.cancelGoal();
                    result.reached= false;
                    as.setAborted(result, "Timeout abort");
                    return;
                }
                
                char blackROIcheck = inBlackROI(goal->x, goal->y, currentPos.x_robot, currentPos.y_robot, fuckin_table);
                if (blackROIcheck!=0)
                {
                    if(blackROIcheck ==  1)
                    {
                        ROS_WARN("Waypoint generated inside black ROI, deleting...");
                        move_base_client.cancelGoal();
                        result.reached = false;
                        as.setAborted(result, "BlackROI abort");
                        return;
                    }else if (blackROIcheck == 2)
                    {
                        ROS_WARN("Movement in blackROI, redirecting...");
                        move_base_client.cancelGoal();
                        move_base_client.sendGoal(move_emergency_goal);
                        move_base_client.sendGoal(move_goal);
                    }
                }
                
                

                if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    result.reached=true;
                    ROS_INFO("NAV OK");
                    as.setSucceeded(result);
                    return;
                }else if(move_base_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    result.reached = false;
                    ROS_ERROR("NAV FAIL");
                    as.setAborted(result);
                    return;
                }
                rate.sleep();
            }
        }

    private:

    robotPos currentRobotPos()
    {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();

        robotPos current {x,y};

        return(current);
    };

    char inBlackROI(double x_goal, double y_goal, double x_robot, double y_robot,blackROI& ROI)
    {

        ROS_INFO("BLACKROI coord: xl:%f xm:%f yl:%f ym:%f", ROI.x_lower, ROI.x_max, ROI.y_lower, ROI.y_max);
        ROS_INFO("GOAL coord:      x:%f  y:%f", x_goal, y_goal);
        ROS_INFO("ROBOT coord :    x:%f  y:%f", x_robot, y_robot);

        if(x_goal>= ROI.x_lower && x_goal<= ROI.x_max && y_goal >= ROI.y_max && y_goal <= ROI.y_lower)
        {
            ROS_WARN("Waypoint is in a BlackROI, movement aborted");
            return 1;
        } else if(x_robot >= ROI.x_lower && x_robot <= ROI.x_max && y_robot >= ROI.y_max && y_robot <= ROI.y_lower)
        {
            ROS_WARN("Robot entered a BlackROI, movement redirected");
            return 2;
        }  else
        {
            return 0;
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "movement_handler");

    MovementHandler move_to_goal("move_to_goal");

    ros::spin();

    return 0;
}

