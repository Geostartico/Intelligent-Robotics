#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <assignment1/WaypointMoveAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h> 

const float MAX_CORRIDOR_X = 5.66; 
const float ANGULAR_VEL = 0.3;
const float LINEAR_VEL = 0.6;
const float ANGLE_CONSIDERED = M_PI_4;

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
    bool custom_mcl_flag;

    public:

        MovementHandler(std::string name, bool flag) :
            as(nh_, name, boost::bind(&MovementHandler::callbackOnGoal, this, _1), false),
            action_name(name),
            move_base_client("move_base", true)
            {
                custom_mcl_flag = flag;
                as.start();
            }
    
        void callbackOnGoal(const assignment1::WaypointMoveGoal::ConstPtr &goal)
        {
            if(custom_mcl_flag) {
                ROS_INFO("INITIAL SPIN TO LOOK AROUND STARTING POS");
                robospin();

                move_base_msgs::MoveBaseGoal  origin_goal;
                origin_goal.target_pose.header.frame_id = "map";
                origin_goal.target_pose.pose.position.x = 0.0;
                origin_goal.target_pose.pose.position.y = 0.0;
                origin_goal.target_pose.pose.position.z = 0.0;
                origin_goal.target_pose.header.stamp = ros::Time::now();

                origin_goal.target_pose.pose.orientation.x = 0.0;
                origin_goal.target_pose.pose.orientation.y = 0.0;
                origin_goal.target_pose.pose.orientation.z = 0.0;
                origin_goal.target_pose.pose.orientation.w = 1.0;

                move_base_client.sendGoal(origin_goal);
                move_base_client.waitForResult(ros::Duration(15.0));

                ROS_INFO("CORRIDOR TRAVERSAL BEGUN - MOVING");
                traverse_corridor();

                custom_mcl_flag = false;
            }

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

            blackROI  table{6.78863,7.78863,-1.82448,-2.82448};
            
            move_base_msgs::MoveBaseGoal  move_emergency_goal;
            move_emergency_goal.target_pose.header.frame_id = "map";
            move_emergency_goal.target_pose.pose.position.x = table.x_max;
            move_emergency_goal.target_pose.pose.position.y = table.y_max;
            move_emergency_goal.target_pose.pose.position.z = 0.0;
            move_emergency_goal.target_pose.header.stamp = ros::Time::now();

            move_emergency_goal.target_pose.pose.orientation.x = 0.0;
            move_emergency_goal.target_pose.pose.orientation.y = 0.0;
            move_emergency_goal.target_pose.pose.orientation.z = 0.0;
            move_emergency_goal.target_pose.pose.orientation.w = 1.0;

            ros::Rate rate(10.0);
            while (ros::ok()) {   
                robotPos currentPos = currentRobotPos();
                
                if ((ros::Time::now() - start_time) >  timeout)
                {
                    ROS_ERROR("Navigation timed out - Cancelling goal...");
                    move_base_client.cancelGoal();
                    result.reached= false;
                    as.setAborted(result, "Timeout abort");
                    robospin();
                    return;
                }
                
                char blackROIcheck = inBlackROI(goal->x, goal->y, currentPos.x_robot, currentPos.y_robot, table);
                if (blackROIcheck!=0)
                {
                    if(blackROIcheck ==  1)
                    {
                        ROS_WARN("Waypoint generated inside black ROI, deleting...");
                        move_base_client.cancelGoal();
                        result.reached = false;
                        as.setAborted(result, "BlackROI abort");
                        return;
                    } else if (blackROIcheck == 2)
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
                    robospin();
                    return;
                } else if(move_base_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    result.reached = false;
                    ROS_ERROR("NAV FAIL");
                    as.setAborted(result);
                    robospin();
                    return;
                }

                rate.sleep();
            }
        }

    private:

    bool is_first_move = true;

    void spin_util(double yaw){
        move_base_msgs::MoveBaseGoal spin;

        spin.target_pose.header.frame_id="base_link";
        spin.target_pose.header.stamp=ros::Time::now();
        spin.target_pose.pose.position.x=0.0;
        spin.target_pose.pose.position.y=0.0;

        tf2::Quaternion q;
        q.setRPY(0,0,yaw);
        spin.target_pose.pose.orientation.x=q.x();
        spin.target_pose.pose.orientation.y=q.y();
        spin.target_pose.pose.orientation.z=q.z();
        spin.target_pose.pose.orientation.w=q.w();

        move_base_client.sendGoal(spin);        

        bool finished = move_base_client.waitForResult(ros::Duration(15.0));
        if(!finished){
            ROS_WARN("Spin unsuccesful");
        } else {
            ROS_WARN("Spinned");
        }
    }

    void robospin() {
        spin_util((2.0/3.0)*M_PI);
        ros::Duration(0.02).sleep();
        spin_util((2.0/3.0)*M_PI);
        ros::Duration(0.02).sleep();
        spin_util((2.0/3.0)*M_PI);
        ros::Duration(0.02).sleep();
        is_first_move = false;
    }

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

    void traverse_corridor() {
        ros::NodeHandle nh;
        ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
        // Wait for the first message on the amcl_pose topic
        ROS_INFO("Waiting for a message on the 'amcl_pose' topic...");
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg;
        while(ros::ok()){
            msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", nh);
            double x_pos = msg->pose.pose.position.x;
            ROS_INFO("pose x: %f", x_pos);
            if(x_pos > MAX_CORRIDOR_X) {
                ROS_INFO("already out");
                return;
            }
            // Extract quaternion
            double x = msg->pose.pose.orientation.x;
            double y = msg->pose.pose.orientation.y;
            double z = msg->pose.pose.orientation.z;
            double w = msg->pose.pose.orientation.w;
            geometry_msgs::Twist vel; 
            vel.linear.x = LINEAR_VEL;
            sensor_msgs::LaserScanConstPtr laserScanMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh);
            float left_mean = 1000;
            float right_mean = 10000;
            float angle_min = laserScanMsg->angle_min;
            float angle_max = laserScanMsg->angle_max;
            float angle_increment = laserScanMsg->angle_increment;
            //ROS_INFO("angle_min: %f, angle_max: %f, angle_increment: %f", angle_min, angle_max, angle_increment);
            int counter_right = 0;
            for(int i = (abs(angle_min) - M_PI_2)/angle_increment; 
                    i * angle_increment - (abs(angle_min) - M_PI_2) < ANGLE_CONSIDERED;
                    i++) {
                //right_mean += laserScanMsg->ranges[i]* cos(abs(angle_min+i*angle_increment));
                float cur = laserScanMsg->ranges[i]* abs(cos(abs(angle_min+i*angle_increment)));
                if(cur<right_mean){
                	right_mean=cur;
                }
                counter_right++;
            }
            //right_mean /= ANGLE_CONSIDERED/angle_increment;

            int counter_left = 0;
            for(int i = (abs(angle_min) + M_PI_2)/angle_increment; 
                    abs(i * angle_increment - abs(angle_min) - M_PI_2)  < ANGLE_CONSIDERED;
                    i--) {
                //left_mean += laserScanMsg->ranges[i]* cos(abs(angle_min+i*angle_increment));
                float cur = laserScanMsg->ranges[i]* abs(cos(abs(angle_min+i*angle_increment)));
                if(cur<left_mean){
                	left_mean=cur;
                }
                counter_left++;
            }
            //left_mean /= ANGLE_CONSIDERED/angle_increment;
            if(right_mean > left_mean)
                vel.angular.z = -ANGULAR_VEL;
            else
                vel.angular.z = ANGULAR_VEL;
            velPub.publish(vel);
            ROS_ERROR("distance_mean left=%f  %d right=%f %d correct: %f", left_mean,counter_left, right_mean, counter_right, ANGLE_CONSIDERED/angle_increment);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "movement_handler");
    
    if (argc < 2) {
        ROS_ERROR("Error: At least one parameter is required.");
        ROS_INFO("Usage: %s <mcl_flag> ", argv[0]);
        return 1; 
    }

    std::string arg1(argv[1]);
    bool mcl_flag = (arg1 == "1" || arg1 == "true");

    MovementHandler move_to_goal("move_to_goal", mcl_flag);

    ros::spin();

    return 0;
}
