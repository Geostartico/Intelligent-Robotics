#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include "assignment2/object_detect.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// CONSTANTS
float TABLE_1_X = 7.82;
float TABLE_1_Y = -1.96;
float TABLE_2_X = 7.82;
float TABLE_2_Y = -3.01;
float DIST      = 2.0;

const float MAX_CORRIDOR_X = 5.66; 
const float ANGULAR_VEL = 0.3;
const float LINEAR_VEL = 0.6;
const float ANGLE_CONSIDERED = M_PI_4;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

bool detection_routine(assignment2::object_detect::Request &req, assignment2::object_detect::Response &res)
{
    if(req.ready == false) {
        ROS_WARN("Detection request is set to false.");
        return false;
    }

    ROS_INFO("Service called. Detection routine starts.");

    MoveBaseClient ac("move_base", true);

    // Wait for the action server to be available
    ROS_INFO("Waiting for the move_base action server to start...");
    ac.waitForServer();
    ROS_INFO("move_base action server is ready.");

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

    // Traverse the corridor before starting movement
    traverse_corridor();

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

        ROS_INFO("[%u/%lu] Sending goal (docking station).", i+1, docks.size());
        ac.sendGoal(goal);
        ac.waitForResult();
        ros::Duration(1.0).sleep();  // delay

        // Corner visit
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

            ROS_INFO("[%u/%lu] Sending goal (corner).", i+1, corns.size());
            ac.sendGoal(goal_corner); 
            ac.waitForResult();
            ros::Duration(1.0).sleep();
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
    ROS_INFO("Object detection service available.");

    ros::spin();
    return 0;
}