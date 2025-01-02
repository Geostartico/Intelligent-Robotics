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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

#include "assignment2/apriltag_detect.h"

/*
 * CONSTANTS
*/
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

/*
 * the following code is separated from the detector as if
 * they were in the same node the service wouldn't respond
 * 
*/

std::vector<int> ids;
std::vector<float> x;
std::vector<float> y;
std::vector<float> z;
std::vector<float> yaw;

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
    for(int i=0; i < ids.size(); i++){
        int id_ = ids[i];
        float x_ = x[i];
        float y_ = y[i];
        float z_ = z[i];
        float yaw_ = yaw[i];
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
    if(!req.ids.empty()){
        ids=req.ids;
        x=req.x;
        y=req.y;
        z=req.z;
        yaw=req.yaw;
    }
    //return the apriltags read up to now
    res.ids = ids;
    res.x = x;
    res.y = y;
    res.z = x;
    res.yaw = yaw;
    if(req.create_collisions){
        add_collision_objects();
    }
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
