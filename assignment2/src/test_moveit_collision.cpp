#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    ROS_INFO("%s",move_group.getPlanningFrame().c_str());
    // Create a PlanningSceneInterface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a CollisionObject message
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame(); // Replace with the appropriate frame ID
    collision_object.id = "boxityBox";

    // Define the shape of the collision object (a box)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3); // Dimensions: X, Y, Z
    primitive.dimensions[primitive.BOX_X]= 0.1; // Dimensions: X, Y, Z
    primitive.dimensions[primitive.BOX_Y]= 1.5; // Dimensions: X, Y, Z
    primitive.dimensions[primitive.BOX_Z]= 0.5; // Dimensions: X, Y, Z

    // Define the pose of the object
    geometry_msgs::Pose box_pose;
    box_pose.position.x = 2.0;
    box_pose.position.y = 2.0;
    box_pose.position.z = 2.0;
    box_pose.orientation.w = 1.0;

    // Add shape and pose to the CollisionObject
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO("Added a collision object into the planning scene.");

    // Sleep to allow RViz to update
    ros::Duration(1000.0).sleep();

    return 0;
}

