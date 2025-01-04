#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <cstdint>
#include <thread>        
#include <chrono>
#include "assignment2/apriltag_detect.h"
#include "assignment2/ObjectMoveAction.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <moveit_msgs/AttachedCollisionObject.h>

const std::set<int> prism { 1, 2, 3};
const std::set<int> cube { 4, 5, 6};
const std::set<int> triangle { 7, 8, 9};
const float APPRO = 0.30;
const float OPENI = 0.10;
const float CLOSEI = 0.02;
const std::vector<double> HOME_JOINT_POSITION = {1.48, 1, 1.5, 1.56, -1, 1.39, 1.5};
typedef actionlib::SimpleActionServer<assignment2::ObjectMoveAction> Server;

class ArmMovementServer{
    
    protected:
    ros::NodeHandle nh_;
    Server as;
    std::string action_name;
    assignment2::ObjectMoveFeedback feedback_;
    assignment2::ObjectMoveResult result_;
    
    public:
        ArmMovementServer(std::string name):as(nh_, name, boost::bind(&ArmMovementServer::movementOnGoal, this, _1), false),
            action_name(name){as.start();}
        
        void movementOnGoal(const assignment2::ObjectMoveGoal::ConstPtr &goal){

            moveit::planning_interface::MoveGroupInterface moveGroup("arm_torso");
            moveGroup.setPoseReferenceFrame("map");
            moveGroup.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;   

            moveArmToHome();
            toggleGripper(true);
            if (goal -> pick){
                
                geometry_msgs::Pose tgtPose = goal->tgt_pose;
                tgtPose.position.z += APPRO;
                moveArmToPoseTGT(moveGroup,plan,tgtPose);
                ros::Duration(3.0).sleep();
                tgtPose.position.z-= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                ros::Duration(2.0).sleep();
                attach_detach_object(goal->tgt_id, true);
                toggleGripper(false);
                tgtPose.position.z+= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                ros::Duration(2.0).sleep();
                moveArmToHome();
                result_.success = true;
            }else {
                geometry_msgs::Pose tgtPose = goal -> tgt_pose;
                tgtPose.position.z -= APPRO;
                moveArmToPoseTGT(moveGroup, plan, tgtPose);
                ros::Duration(3.0).sleep();
                tgtPose.position.z+= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                ros::Duration(2.0).sleep();
                attach_detach_object(goal->tgt_id, false);
                toggleGripper(true);
                tgtPose.position.z-= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                ros::Duration(2.0).sleep();
                moveArmToHome();
                result_.success = true;
            }
        }

    private:

        std::string get_name(int apriltag){
            if(prism.find(apriltag)!=prism.end()){
                return "Hexagon";
            }
            if(prism.find(apriltag)!=prism.end()){
                return "Triangle";
            }
            if(prism.find(apriltag)!=prism.end()){
                return "cube";
            }
            return("error");
        }
        void attach_detach_object(int id, bool attach){
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            ros::ServiceClient gazebo_attach = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/" + attach ? "attach" : "detach");
            gazebo_ros_link_attacher::Attach attachRequest;
            attachRequest.request.model_name_1 = "tiago";
            attachRequest.request.link_name_1 = "arm_7_link";
            std::string objname = get_name(id) + "_" + std::to_string(id);
            attachRequest.request.model_name_2 = objname;
            attachRequest.request.link_name_2 = objname +"_link" ;
            //if(gazebo_attach.call(attachRequest)){
            //    ROS_INFO("object attached/detached correctly in gazebo");
            //}
            //else{
            //    ROS_ERROR("unable to complete attaching/detaching action in gazebo");
            //}
            //moveit part
            std::string object_id = "box_april_"+std::to_string(id);
            std::vector<std::string> objects_query = {object_id};
            moveit_msgs::CollisionObject coll = planning_scene_interface.getObjects(objects_query)[0];
            std::string link_name = "arm_7_link";
            std::vector<std::string> touch_links = {
                "arm_1_link",
                "arm_2_link",
                "arm_3_link",
                "arm_4_link",
                "arm_5_link",
                "arm_6_link",
                "arm_7_link",
                "gripper_left_finger_link",
                "gripper_right_finger_link",
            };
            moveit_msgs::AttachedCollisionObject attached_object;
            attached_object.link_name = link_name;
            attached_object.object = coll;
            attached_object.touch_links = touch_links;
            if(attach){
                attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
                planning_scene_interface.applyAttachedCollisionObject(attached_object);
            }
            else{
                attached_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
                planning_scene_interface.applyAttachedCollisionObject(attached_object);
            }
        }


        void moveArmToHome() {
            moveit::planning_interface::MoveGroupInterface move_group("arm"); 
            move_group.setJointValueTarget(HOME_JOINT_POSITION);
            moveit::planning_interface::MoveGroupInterface::Plan home_plan;
            if (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                move_group.execute(home_plan);
                ROS_INFO("Home plan OK");
            } else {
                ROS_WARN("Home Plan Failed");
            }
        }

        bool toggleGripper(bool open){
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper("/parallel_gripper_controller/follow_joint_trajectory", true);
            gripper.waitForServer();

            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
            goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
            trajectory_msgs::JointTrajectoryPoint point;

            if(open){
                point.positions.push_back(OPENI);
                point.positions.push_back(OPENI);
            }else{
                point.positions.push_back(CLOSEI);
                point.positions.push_back(CLOSEI);
            }
            point.time_from_start = ros::Duration(1.5);
            goal.trajectory.points.push_back(point);

            gripper.sendGoal(goal);
            if (gripper.waitForResult()) {
            ros::Duration(1.0).sleep();
            ROS_INFO("Gripper moved successfully.");
            return true;
            }
            ROS_ERROR("gripper failure");
            return false;
        }

        bool moveArmToPoseTGT(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& tgt){
            
            moveGroup.setPoseTarget(tgt);
            moveit::core::MoveItErrorCode planResult = moveGroup.plan(plan);
            if (planResult){
                ROS_INFO("arm moving to tgt pose");
                moveGroup.move();
                ros::Duration(5.0).sleep();
                return true;
            }

            ROS_ERROR("Error in movement to TGT");
            return false;
        }

        void moveLinearTGT(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& tgt){
            moveit_msgs::RobotTrajectory path;
            //float pathCartesian = moveGroup.computeCartesianPath(std::vector<geometry_msgs::Pose>{tgt}, 0.03,path); //compiles on local/updated ROS versions
            float pathCartesian = moveGroup.computeCartesianPath(std::vector<geometry_msgs::Pose>{tgt}, 0.03, 0,path); //compiles on vlab, deprecated.
            if (pathCartesian > 0){
                ROS_INFO("Arm in cartesian movement");
                plan.trajectory_=path;
                moveGroup.execute(plan);
                ros::Duration(5.0).sleep();
                return;
            }
            ROS_ERROR("no cartesian path available");
            return;
        }   
};




int main(int argc, char** argv) {
    ros::init(argc, argv, "ArmMovementServer");
    ros::AsyncSpinner spinner(1);
    spinner.start();


    ArmMovementServer move_object("move_object");
    ros::waitForShutdown();
    return 0;
}
