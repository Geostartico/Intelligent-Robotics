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

#include "assignment1/apriltag_detect.h"

/*
 * the following code is separated from the detector as if
 * they were in the same node the service wouldn't respond
 * 
*/

std::vector<int> ids;
std::vector<float> x;
std::vector<float> y;

bool get_apriltags(assignment1::apriltag_detect::Request &req, assignment1::apriltag_detect::Response &res){
    //the program saves the apriltags received from the detector
    if(!req.ids.empty()){
        ids=req.ids;
        x=req.x;
        y=req.y;
    }
    //return the apriltags read up to now
    res.ids = ids;
    res.x = x;
    res.y = y;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "locate_apriltag_service");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("locate_apriltag");

    ros::Subscriber tag_subscriber;

    ros::ServiceServer service = nh.advertiseService("apriltags_detected_service", get_apriltags);
    ROS_INFO_STREAM("locate_apriltag");
    ros::spin(); 
    return 0;
}
