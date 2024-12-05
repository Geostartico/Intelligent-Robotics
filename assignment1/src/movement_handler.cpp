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
    double xl;
    double xm;
    double yl;
    double ym;
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

            blackROI  fuckin_table{-0.267452, 1.732548, -1.918757, 0.081243};
            // blackROI  fuckin_table{100.267452, 101.732548, 100.918757, 103.081243};
            
            ros::Rate rate(10.0);
            while (ros::ok())
            {
                if ((ros::Time::now() - start_time) >  timeout)
                {
                    ROS_ERROR("Navigation timed out - Cancelling goal...");
                    move_base_client.cancelGoal();
                    result.reached= false;
                    as.setAborted(result, "Timeout abort");
                    return;
                }
                
                
                if(inBlackROI(goal->x, goal->y, fuckin_table)){
                    move_base_client.cancelGoal();
                    result.reached = false;
                    as.setAborted(result, "BlackROI abort");
                    return;
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
    bool inBlackROI(double x_goal, double y_goal, blackROI& ROI)
    {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);

        double x_robot = transform.getOrigin().x();
        double y_robot = transform.getOrigin().y();

        ROS_INFO("BLACKROI coord: xl:%f xm:%f yl:%f ym:%f", ROI.xl, ROI.xm, ROI.yl, ROI.ym);
        ROS_INFO("GOAL coord:      x:%f  y:%f", x_goal, y_goal);
        ROS_INFO("ROBOT coord :    x:%f  y:%f", x_robot, y_robot);

        if(x_goal>= ROI.xl && x_goal<= ROI.xm && y_goal >= ROI.yl && y_goal <= ROI.ym)
        {
            ROS_WARN("Waypoint is in a BlackROI, movement aborted");
            return true;
        } else if(x_robot >= ROI.xl && x_robot <= ROI.xm && y_robot >= ROI.yl && y_robot <= ROI.ym)
        {
            ROS_WARN("Robot entered a BlackROI, movement aborted");
            return true;
        }  else
        {
            return false;
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "movement_handler");

    MovementHandler move_to_goal("move_to_goal");

    ros::spin();

    return 0;
}

