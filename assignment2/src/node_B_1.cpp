#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>



#include <actionlib/client/simple_action_client.h>

#include <map>

#include "assignment2/apriltag_detect.h"


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <map>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

#include "assignment2/apriltag_detect.h"

#include <actionlib/client/simple_action_client.h>

#include <map>

#include "assignment2/apriltag_detect.h"

//after how many iterations to send the positions
int NUM_ITER_SEND = 10;
std::set<int> prism { 1, 2, 3};
std::set<int> cube { 4, 5, 6};
std::set<int> triangle { 7, 8, 9};
float PRISM_HEIGHT = 0.2;
float PRISM_RADIUS = 0.05;
float CUBE_SIDE = 0.05;
float TRIANGLE_BASE = 0.05;
float TRIANGLE_HEIGHT = 0.035;
float TRIANGLE_LENGTH = 0.07;
float TABLE_SIDE = 0.9;
float TABLE_HEIGHT = 0.775;
float TABLE_1_X = 7.82;
float TABLE_1_Y = -1.96;
float TABLE_2_X = 7.82;
float TABLE_2_Y = -3.01;

struct aprilmean{
    float x;
    float y;
    float z;
    int id;
    float yaw;
};

std::map<int,aprilmean> apriltags_detected;

void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg){
    //we need the position in respect to the map
    std::string target_frame = "map";
    std::string source_frame = msg->header.frame_id;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
        ros::Duration(0.001).sleep();

    geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));

    geometry_msgs::PoseStamped pos_in;

    for(int i = 0; i < msg->detections.size(); ++i){
	ROS_INFO("DETECTED ID: %d",msg->detections.at(i).id[0]);
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
        double roll, pitch, yaw;
        aprilmean tmp;
        tmp.x = pos_out.pose.position.x;
        tmp.y = pos_out.pose.position.y;
        tmp.z = pos_out.pose.position.z;
        tmp.id = msg->detections.at(i).id[0];
        tf2::Quaternion quat;
        tf2::convert(pos_out.pose.orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        tmp.yaw = yaw;
        apriltags_detected[tmp.id] = (tmp);
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
    target_point.point.x = 2.0; 
    target_point.point.y = 0.0; 
    target_point.point.z = -0.3;

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
void move_torso(){
    // Create client the torso_controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torsoClient("/torso_controller/follow_joint_trajectory", true);
    ROS_INFO("waiting for torso server");
    torsoClient.waitForServer();

    // Set joint values
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(1.0);
    point.time_from_start = ros::Duration(1.0);
    goal.trajectory.points.push_back(point);

    // Send goal and check the result
    torsoClient.sendGoal(goal);
    if (torsoClient.waitForResult()) {
        //ros::Duration(DELAY).sleep();
        ROS_INFO("The torso moved successfully.");
    }
    else
        ROS_ERROR("An error occurred while moving the torso.");
}

/*
 * the following code is separated from the detector as if
 * they were in the same node the service wouldn't respond
 * 
*/


void add_tables(){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.operation = collision_object1.ADD;
    collision_object1.id = "table_1";
    collision_object1.header.frame_id = "/map"; // Replace with the appropriate frame ID
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[primitive1.BOX_X]= TABLE_SIDE; // Dimensions: X, Y, Z
    primitive1.dimensions[primitive1.BOX_Y]= TABLE_SIDE; // Dimensions: X, Y, Z
    primitive1.dimensions[primitive1.BOX_Z]= TABLE_HEIGHT; // Dimensions: X, Y, Z
    geometry_msgs::Pose box_pose1;
    box_pose1.position.x = TABLE_1_X;
    box_pose1.position.y = TABLE_1_Y;
    box_pose1.position.z = TABLE_HEIGHT/2;
    box_pose1.orientation.w = 1.0;
    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(box_pose1);
    collision_objects.push_back(collision_object1);

    moveit_msgs::CollisionObject collision_object2;
    collision_object2.operation = collision_object2.ADD;
    collision_object2.id = "table_2";
    collision_object2.header.frame_id = "/map"; // Replace with the appropriate frame ID
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive2.BOX_X]= TABLE_SIDE; // Dimensions: X, Y, Z
    primitive2.dimensions[primitive2.BOX_Y]= TABLE_SIDE; // Dimensions: X, Y, Z
    primitive2.dimensions[primitive2.BOX_Z]= TABLE_HEIGHT; // Dimensions: X, Y, Z
    geometry_msgs::Pose box_pose2;
    box_pose2.position.x = TABLE_2_X;
    box_pose2.position.y = TABLE_2_Y;
    box_pose2.position.z = TABLE_HEIGHT/2;
    box_pose2.orientation.w = 1.0;
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_objects.push_back(collision_object2);
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void add_collision_objects(){
    add_tables();
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    ROS_INFO("%s",move_group.getPlanningFrame().c_str());
    // Create a PlanningSceneInterface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a CollisionObject message
    //collision_object.header.frame_id = move_group.getPlanningFrame(); // Replace with the appropriate frame ID

    // Add shape and pose to the CollisionObject

    // Add the collision object to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for(auto tag : apriltags_detected){
        int id_ = tag.second.id;
        float x_ = tag.second.x;
        float y_ = tag.second.y;
        float z_ = tag.second.z;
        float yaw_ = tag.second.yaw;
        float height, width, length;
        moveit_msgs::CollisionObject collision_object;
        collision_object.operation = collision_object.ADD;
        collision_object.id = "box_april_"+std::to_string(id_);
        collision_object.header.frame_id = "/map";
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        if(prism.find(id_) != prism.end()){
            z_ -= PRISM_HEIGHT/2;
            height = PRISM_HEIGHT;
            width = PRISM_RADIUS;
            length = PRISM_RADIUS;
        }
        else if(cube.find(id_) != cube.end()){
            z_ -= CUBE_SIDE/2;
            height = CUBE_SIDE;
            width = CUBE_SIDE;
            length = CUBE_SIDE;
        }
        else if(triangle.find(id_) != triangle.end()){
            z_ -= TRIANGLE_HEIGHT/2;
            x_ += sin(yaw_) * TRIANGLE_BASE/4;
            y_ -= cos(yaw_) * TRIANGLE_BASE/4;
            height = TRIANGLE_HEIGHT;
            width = TRIANGLE_LENGTH;
            length = TRIANGLE_BASE;
        }
        else{
            continue;
        }
        primitive.dimensions[primitive.BOX_X]= width; // Dimensions: X, Y, Z
        primitive.dimensions[primitive.BOX_Y]= length; // Dimensions: X, Y, Z
        primitive.dimensions[primitive.BOX_Z]= height; // Dimensions: X, Y, Z
        // Define the pose of the object
        geometry_msgs::Pose box_pose;
        box_pose.position.x = x_;
        box_pose.position.y = y_;
        box_pose.position.z = z_;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw_);
        box_pose.orientation.x = quaternion.x();
        box_pose.orientation.y = quaternion.y();
        box_pose.orientation.z = quaternion.z();
        box_pose.orientation.w = quaternion.w();
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_objects.push_back(collision_object);
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool get_apriltags(assignment2::apriltag_detect::Request &req, assignment2::apriltag_detect::Response &res){
    //the program saves the apriltags received from the detector
    ros::NodeHandle nh;
    const apriltag_ros::AprilTagDetectionArray::ConstPtr msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("tag_detections", nh);
    detectionCallback(msg);
    for(auto el : apriltags_detected){
        res.ids.push_back(el.second.id);
        res.x.push_back(el.second.x);
        res.y.push_back(el.second.y);
        res.z.push_back(el.second.z);
        res.yaw.push_back(el.second.yaw);
    }
    if(req.create_collisions){
        add_collision_objects();
    }
    return true;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "locate_apriltag");
    ros::NodeHandle nh;
    lookDown();
    move_torso();
    ROS_INFO_STREAM("locate_apriltag");

    ros::Subscriber tag_subscriber;

    //tag_subscriber = nh.subscribe("tag_detections", 10, detectionCallback);
    ros::ServiceServer service = nh.advertiseService("apriltags_detected_service", get_apriltags);
    ROS_INFO_STREAM("locate_apriltag");
    ros::spin(); 
    return 0;
}
