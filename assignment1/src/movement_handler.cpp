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
#include <cmath>

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

bool is_closer(move_base_msgs::MoveBaseGoal current, move_base_msgs::MoveBaseGoal next, move_base_msgs::MoveBaseGoal target){
	float current_x = current.target_pose.pose.position.x;
	float current_y = current.target_pose.pose.position.y;
	float next_x = next.target_pose.pose.position.x;
	float next_y = next.target_pose.pose.position.y;
	float target_x = target.target_pose.pose.position.x;
	float target_y = target.target_pose.pose.position.y;
	float distance_cur = std::pow(std::pow(current_x,2)-std::pow(target_x,2),2)+std::pow(std::pow(current_y,2)-std::pow(target_y,2),2);
	float distance_next = std::pow(std::pow(next_x,2)-std::pow(target_x,2),2)+std::pow(std::pow(next_y,2)-std::pow(target_y,2),2);
	return distance_cur>distance_next;
}
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
                robospin();

                //origin position is saved to reset robot initial state after spinning, avoids collision while running mcl
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

                traverse_corridor();

                custom_mcl_flag = false;
            };
            //loads waypoint from action and stores it in a variable, sent to movebase and used for redirections
            move_base_msgs::MoveBaseGoal  move_goal;
            move_goal.target_pose.header.frame_id = "map";
            move_goal.target_pose.pose.position.x = goal->x;
            move_goal.target_pose.pose.position.y = goal->y;
            move_goal.target_pose.header.stamp = ros::Time::now();

            move_goal.target_pose.pose.orientation.w = 1.0;

            move_base_client.sendGoal(move_goal);

            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(20);

            /*coordinates obtained from RViz, following the map reference, 2m size to avoid collision.
            *saving bottom and top left corners as emergency waypoints allow redirection on the outer perimeter of the ROI
            */
             blackROI  table{6.28863,8.28863,-1.07448,-3.57448};

            ros::Rate rate(10.0);
            while (ros::ok()) {   
                robotPos currentPos = currentRobotPos();
                
                if ((ros::Time::now() - start_time) >  timeout)
                {
                    move_base_client.cancelGoal();
                    result.reached= false;
                    as.setAborted(result, "Timeout abort");
                    robospin();
                    return;
                }
                
                if(inBlackROI(goal->x, goal->y, currentPos.x_robot, currentPos.y_robot, table)) return;
                
                        //returns for node_B
                if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    result.reached=true;
                    as.setSucceeded(result);
                    robospin();
                    return;
                } else if(move_base_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    result.reached = false;
                    as.setAborted(result);
                    robospin();
                    return;
                }

                rate.sleep();
            }
        }

    private:

    bool is_first_move = true;
    
    //spin utils ans robospin separated, the first one executes a defined angle position, robospin sums angles to perform a 360° rotation. details in report
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
    }

    void robospin() {
        
        for (size_t i = 0; i < 3; i++)
        {
            spin_util((2.0/3.0)*M_PI);
            ros::Duration(0.02).sleep();
        }
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

    bool inBlackROI(double x_goal, double y_goal, double x_robot, double y_robot,blackROI& ROI)
    {

        if(x_goal>= ROI.x_lower && x_goal<= ROI.x_max && y_goal >= ROI.y_max && y_goal <= ROI.y_lower) {
            
            result.reached = false;
            move_base_client.cancelGoal();
            as.setAborted(result, "Spawn in roi abort");
            return true;
        } 
        else if(x_robot >= ROI.x_lower && x_robot <= ROI.x_max && y_robot >= ROI.y_max && y_robot <= ROI.y_lower) {
            move_base_client.cancelGoal();
            bool finished;

            move_base_msgs::MoveBaseGoal  move_emergency_goal_up;
            move_emergency_goal_up.target_pose.header.frame_id = "map";
            move_emergency_goal_up.target_pose.pose.position.x = ROI.x_max;
            move_emergency_goal_up.target_pose.pose.position.y = ROI.y_lower;
            move_emergency_goal_up.target_pose.header.stamp = ros::Time::now();

            move_emergency_goal_up.target_pose.pose.orientation.w = 1.0;

            move_base_msgs::MoveBaseGoal  move_emergency_goal_down;
            move_emergency_goal_down.target_pose.header.frame_id = "map";
            move_emergency_goal_down.target_pose.pose.position.x = ROI.x_max;
            move_emergency_goal_down.target_pose.pose.position.y = ROI.y_max;
            move_emergency_goal_down.target_pose.header.stamp = ros::Time::now();

            move_emergency_goal_down.target_pose.pose.orientation.w = 1.0;

            move_base_msgs::MoveBaseGoal  og_move_goal;
            og_move_goal.target_pose.header.frame_id = "map";
            og_move_goal.target_pose.pose.position.x = x_goal;
            og_move_goal.target_pose.pose.position.y = y_goal;
            og_move_goal.target_pose.header.stamp = ros::Time::now();

            og_move_goal.target_pose.pose.orientation.w = 1.0;

            /*if robot moves inside of the ROI during navigation, it will move to the closest waypoint first.
            *left picked to avoid getting stuck in the wall. take top or bottom first relating to current y position
            */
            if(y_robot > (ROI.y_lower+ROI.y_max)/2) {   
                move_base_client.sendGoal(move_emergency_goal_up);
                finished = move_base_client.waitForResult(ros::Duration(15.0));
                ros::Duration(0.02).sleep();
                if(is_closer(move_emergency_goal_up, move_emergency_goal_down, og_move_goal)){
                    move_base_client.sendGoal(move_emergency_goal_down);
                    finished = move_base_client.waitForResult(ros::Duration(15.0));
                    ros::Duration(0.02).sleep();
                }
            } 
            else {
                move_base_client.sendGoal(move_emergency_goal_down);
                finished = move_base_client.waitForResult(ros::Duration(15.0));
                ros::Duration(0.02).sleep();
                if(is_closer(move_emergency_goal_down, move_emergency_goal_up, og_move_goal)){
                    move_base_client.sendGoal(move_emergency_goal_up);
                    finished = move_base_client.waitForResult(ros::Duration(15.0));
                    ros::Duration(0.02).sleep();
                }
            }
            move_base_client.sendGoal(og_move_goal);
            finished = move_base_client.waitForResult(ros::Duration(15.0));
            ros::Duration(0.02).sleep();
        }
        return false; 
    };

    void traverse_corridor() {
        ros::NodeHandle nh;
        ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
        // Wait for the first message on the amcl_pose topic
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg;
        while(ros::ok()){
            msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", nh);
            double x_pos = msg->pose.pose.position.x;
            if(x_pos > MAX_CORRIDOR_X) {
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
            int counter_right = 0;
            for(int i = (abs(angle_min) - M_PI_2)/angle_increment; 
                    i * angle_increment - (abs(angle_min) - M_PI_2) < ANGLE_CONSIDERED;
                    i++) {
                float cur = laserScanMsg->ranges[i]* abs(cos(abs(angle_min+i*angle_increment)));
                if(cur<right_mean){
                	right_mean=cur;
                }
                counter_right++;
            }
            int counter_left = 0;
            for(int i = (abs(angle_min) + M_PI_2)/angle_increment; 
                    abs(i * angle_increment - abs(angle_min) - M_PI_2)  < ANGLE_CONSIDERED;
                    i--) {
                float cur = laserScanMsg->ranges[i]* abs(cos(abs(angle_min+i*angle_increment)));
                if(cur<left_mean){
                	left_mean=cur;
                }
                counter_left++;
            }
            if(right_mean > left_mean)
                vel.angular.z = -ANGULAR_VEL;
            else
                vel.angular.z = ANGULAR_VEL;
            velPub.publish(vel);
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
