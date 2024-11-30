#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <map>

#include "assignment1/apriltag_detect.h"


std::map<int, std::vector<geometry_msgs::PoseStamped>> apriltags_detected;

void detectionCallbackTF2(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  std::string target_frame = "base_link";
  std::string source_frame = msg->header.frame_id;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
    ros::Duration(0.5).sleep();

  geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  
  //Transform available
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

      ROS_INFO_STREAM("Obj with ID: " << msg->detections.at(i).id[0]);
      ROS_INFO_STREAM("Original pose\n" << pos_in);
      ROS_INFO_STREAM("Transformed pose\n" << pos_out);
      if(apriltags_detected.find((int)(msg->detections.at(i).id[0])) == apriltags_detected.end()){
          std::vector<geometry_msgs::PoseStamped> tmp = {pos_out};
          apriltags_detected[msg->detections.at(i).id[0]] = tmp;
      }
      else
          apriltags_detected.at(msg->detections.at(i).id[0]).push_back(pos_out);

  }
}
bool get_apriltags(assignment1::apriltag_detect::Request &req, assignment1::apriltag_detect::Response &res){
    for(auto const &april : apriltags_detected){
        res.id.push_back(april.first);
        float x_ = 0;
        float y_ = 0;
        int counter = 0;

        for(auto const &position : april.second){
            counter++;
            x_ += position.pose.position.x;
            y_ += position.pose.position.y;
        }
        res.x.push_back(x_/counter);
        res.y.push_back(y_/counter);
    }
    apriltags_detected.clear();
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "locate_apriltag");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("locate_apriltag");

  ros::Subscriber tag_subscriber;
  
  tag_subscriber = nh.subscribe("tag_detections", 1000, detectionCallbackTF2);
  ros::ServiceServer service = nh.advertiseService("apriltags_detected_service", get_apriltags);

  ros::spin();
  return 0;
}
