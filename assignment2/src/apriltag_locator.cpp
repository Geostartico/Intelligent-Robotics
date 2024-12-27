#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <control_msgs/PointHeadAction.h>


#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <map>

#include "assignment2/apriltag_detect.h"

//after how many iterations to send the positions
int NUM_ITER_SEND = 10;

struct aprilmean{
    float x;
    float y;
    float z;
    float yaw;
    int counter;
};
std::map<int, aprilmean> apriltags_detected;

//function to send apriltags to the service
bool send_apriltags(){
    ros::NodeHandle nh_;
    assignment2::apriltag_detect req;
    for(auto const &april : apriltags_detected){
        req.request.ids.push_back(april.first);
        float x_ = april.second.x/april.second.counter;
        float y_ = april.second.y/april.second.counter;
        float z_ = april.second.z/april.second.counter;
        float yaw_ = april.second.yaw/april.second.counter;
        req.request.x.push_back(x_);
        req.request.y.push_back(y_);
        req.request.z.push_back(z_);
        req.request.yaw.push_back(yaw_);
    }
    ros::ServiceClient ad_client = nh_.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");
    ad_client.call(req);
    return true;
}

int n = 0;
void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg){
    //we need the position in respect to the map
    std::string target_frame = "map";
    std::string source_frame = msg->header.frame_id;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
        ros::Duration(0.05).sleep();

    geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));

    geometry_msgs::PoseStamped pos_in;

    for(int i = 0; i < msg->detections.size(); ++i){
        geometry_msgs::PoseStamped pos_out;
        pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
        pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
        pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
        pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
        pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
        pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
        pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
        pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;

        tf2::doTransform(pos_in, pos_out, transformed);

        //insert the detection in the global vector
        if(apriltags_detected.find((int)(msg->detections.at(i).id[0])) == apriltags_detected.end()){
            aprilmean tmp;
            tmp.x = pos_out.pose.position.x;
            tmp.y = pos_out.pose.position.y;
            tmp.counter = 1;
            apriltags_detected[msg->detections.at(i).id[0]] = tmp;
        }
        else{
            apriltags_detected.at(msg->detections.at(i).id[0]).x += pos_out.pose.position.x;
            apriltags_detected.at(msg->detections.at(i).id[0]).y += pos_out.pose.position.y;
            apriltags_detected.at(msg->detections.at(i).id[0]).z += pos_out.pose.position.x;
            apriltags_detected.at(msg->detections.at(i).id[0]).yaw += pos_out.pose.orientation.w;
            apriltags_detected.at(msg->detections.at(i).id[0]).counter++;
        }

    }
    n++;
    //send detections
    if(NUM_ITER_SEND-n<=0){
        n=0;
        send_apriltags();
    }
}
//function to move the head down in order to see the apriltags
void lookDown() {
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> client("/head_controller/point_head_action", true);

    ROS_INFO("Waiting for the action server to start...");
    client.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");

    control_msgs::PointHeadGoal goal;

    //initialize the position the head must look at
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
        ROS_ERROR("Head movement did not finish before the timeout");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "locate_apriltag");
    ros::NodeHandle nh;
    lookDown();
    ROS_INFO_STREAM("locate_apriltag");

    ros::Subscriber tag_subscriber;

    tag_subscriber = nh.subscribe("tag_detections", 10, detectionCallback);
    ROS_INFO_STREAM("locate_apriltag");
    ros::spin(); 
    return 0;
}
