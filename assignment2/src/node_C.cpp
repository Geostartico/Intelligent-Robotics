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
//const std::vector<double> HOME_JOINT_POSITION_MEH = {1.48, 1, 1.5, 1.56, -1, 1.39, 1.5};
//const std::vector<double> HOME_JOINT_POSITION = {0.200, 1, -1.68, 1.533, -2, 1.39, 0.10};
// best for now const std::vector<double> HOME_JOINT_POSITION = {1.7, 1, 1.5, 2.25, -0.7, 1.2, 0.10};
const std::vector<double> HOME_JOINT_POSITION = {0.07, 0.978, -2.366, 1.037, -1.238, 1.245, -0.227};
const std::vector<double> TUCKED_JOINT_POSITION = {0.200, -1.339, -0.200, 1.938, -1.570, 1.370, 0};
typedef actionlib::SimpleActionServer<assignment2::ObjectMoveAction> Server;
const std::map<int, std::string> id2model = {
    {1, "Hexagon"},
    {2, "Hexagon_2"},
    {3, "Hexagon_3"},
    {4, "cube"},
    {5, "cube_5"},
    {6, "cube_6"},
    {7, "Triangle"},
    {8, "Triangle_8"},
    {9, "Triangle_9"},
};

class ArmMovementServer{
    
    protected:
    ros::NodeHandle nh_;
    Server as;
    std::string action_name;
    assignment2::ObjectMoveFeedback feedback_;
    assignment2::ObjectMoveResult result_;
    
    public:
        ArmMovementServer(std::string name):as(nh_, name, boost::bind(&ArmMovementServer::movementOnGoal, this, _1), false),
            action_name(name) {
                as.start();
                moveJointToPos(HOME_JOINT_POSITION);
                }
        
        void movementOnGoal(const assignment2::ObjectMoveGoal::ConstPtr &goal){

            moveit::planning_interface::MoveGroupInterface moveGroup("arm_torso");
            moveGroup.setPoseReferenceFrame("map");
            moveGroup.setPlanningTime(120.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;   

            if(goal->pick)
                toggleGripper(true);

            if (goal -> pick){
                std::vector<std::string> tmp = {"box_april_"+std::to_string(goal->tgt_id)};
                std::string object_id = "box_april_"+std::to_string(goal->tgt_id);
                std::vector<std::string> objects_query = {object_id};
                ROS_INFO("Object: %s",objects_query[0].c_str());
                std::map<std::string, moveit_msgs::CollisionObject> colls = planningSceneInterface.getObjects(objects_query);
                ROS_INFO("Objects: %d",int(colls.size()));
                moveJointToPos(HOME_JOINT_POSITION);
                geometry_msgs::Pose tgtPose = goal->tgt_pose;
                tgtPose.position.z += APPRO;
                if(!moveArmToPoseTGT(moveGroup,plan,tgtPose)){
                    moveLinearTGT(moveGroup,plan,tgtPose);
                }
                remove_padding(goal->tgt_id);
                // planningSceneInterface.removeCollisionObjects(tmp);
                //ros::Duration(3.0).sleep();
                tgtPose.position.z-= APPRO;
                tgtPose.position.z+= 0.10;
                moveLinearTGT(moveGroup,plan,tgtPose);
		        //tmp[0] =  "table_2";
                //planningSceneInterface.removeCollisionObjects(tmp);
                //ros::Duration(2.0).sleep();
                toggleGripper(false);
                ros::Duration(1.0).sleep();
                attach_detach_object_moveit(goal->tgt_id,colls[object_id], true);
                attach_detach_object_gazebo(goal->tgt_id, true);
                ros::Duration(2.0).sleep();
                tgtPose.position.z+= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                //ros::Duration(2.0).sleep();
                moveJointToPos(HOME_JOINT_POSITION);
                //moveJointToPos(TUCKED_JOINT_POSITION);
                result_.success = true;
                feedback_.status = {"Robot picked the piece."};
                as.publishFeedback(feedback_);
                as.setSucceeded(result_);
            } 
            else {
                moveJointToPos(HOME_JOINT_POSITION);
                geometry_msgs::Pose tgtPose = goal -> tgt_pose;
                tgtPose.position.z += APPRO;
                //moveArmToPoseTGT(moveGroup, plan, tgtPose);
		        moveArmToPoseTGT(moveGroup, plan, tgtPose);
                //ros::Duration(3.0).sleep();
                tgtPose.position.z-= APPRO;
                tgtPose.position.z+= 0.15;
                moveLinearTGT(moveGroup,plan,tgtPose);
                //ros::Duration(2.0).sleep();
                attach_detach_object_moveit(goal->tgt_id,moveit_msgs::CollisionObject{}, false);
                attach_detach_object_gazebo(goal->tgt_id, false);
                toggleGripper(true);
                ros::Duration(3.0).sleep();
                tgtPose.position.z+= APPRO;
                moveLinearTGT(moveGroup,plan,tgtPose);
                //ros::Duration(2.0).sleep();
                moveJointToPos(HOME_JOINT_POSITION);
                //moveJointToPos(TUCKED_JOINT_POSITION);
                result_.success = true;
                feedback_.status = {"Robot placed the piece."};
                as.publishFeedback(feedback_);
                as.setSucceeded(result_);
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

   
        // TO DO: CAMBIA I NOMI DEI MODELLI
        void attach_detach_object_gazebo(int id, bool attach){
            ros::ServiceClient gazebo_attach = nh_.serviceClient<gazebo_ros_link_attacher::Attach>(
                attach ? "/link_attacher_node/attach" : "/link_attacher_node/detach");

            gazebo_attach.waitForExistence();
            
            gazebo_ros_link_attacher::Attach attachRequest;
            attachRequest.request.model_name_1 = "tiago";
            attachRequest.request.link_name_1 = "arm_7_link";
            if (id2model.find(id) == id2model.end()) {
                ROS_ERROR("Invalid ID: %d", id);
                return;
            }
            attachRequest.request.model_name_2 = id2model.at(id);
            attachRequest.request.link_name_2 = id2model.at(id) +"_link" ;
            
            if(gazebo_attach.call(attachRequest)){
                ROS_INFO("object attached/detached correctly in gazebo");
            }
            else{
                ROS_ERROR("unable to complete attaching/detaching action in gazebo");
            }
        }

        void attach_detach_object_moveit(int id, moveit_msgs::CollisionObject coll, bool attach){
            //moveit part
            // if(attach){
            //     coll.operation = coll.ADD;
            //     planning_scene_interface.applyCollisionObject(coll);
            // }
            moveit::planning_interface::MoveGroupInterface moveGroup("arm_torso");
            std::string object_id = "box_april_"+std::to_string(id);
            //std::vector<std::string> objects_query = {object_id};
            //moveit_msgs::CollisionObject coll = planning_scene_interface.getObjects(objects_query)[0];
            std::string link_name = "arm_7_link";
            std::vector<std::string> touch_links = {
               // "arm_1_link",
               // "arm_2_link",
               // "arm_3_link",
               // "arm_4_link",
               // "arm_5_link",
               // "arm_6_link",
                "arm_7_link",
                "gripper_left_finger_link",
                "gripper_right_finger_link"
            };
            //moveit_msgs::AttachedCollisionObject attached_object;
            //attached_object.link_name = link_name;
            //attached_object.object = coll;
            //attached_object.touch_links = touch_links;
            if(attach){
                //attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
                //planning_scene_interface.applyAttachedCollisionObject(attached_object);
                moveGroup.attachObject(object_id, link_name, touch_links);
            }
            else{
                //attached_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
                //planning_scene_interface.applyAttachedCollisionObject(attached_object);
                moveGroup.detachObject(object_id);
            }
        }

        void moveJointToPos(const std::vector<double> joint_positions) {
            moveit::planning_interface::MoveGroupInterface move_group("arm"); 
            move_group.setJointValueTarget(joint_positions);
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
                // ros::Duration(1.0).sleep();
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
                // ros::Duration(5.0).sleep();
                return true;
            }

            ROS_ERROR("Error in movement to TGT, %d", planResult.val);
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
                // ros::Duration(5.0).sleep();
                return;
            }
            ROS_ERROR("no cartesian path available");
            return;
        }   
        void remove_padding(int id){
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;   
            auto vec = planningSceneInterface.getObjects({"box_april_"+std::to_string(id)});
            std::vector<moveit_msgs::CollisionObject> colls;
            for(auto el : vec){
		ROS_ERROR("OBJECT:%s",el.second.id.c_str());
                auto elcol = el.second.primitives[0];
                MOVEIT_MSGS_MESSAGE_MOVEITERRORCODES_H
		ROS_ERROR("DIMS:%f",el.second.primitives[0].dimensions[elcol.BOX_X]);
                elcol.dimensions[elcol.BOX_X] -= 0.05;
                elcol.dimensions[elcol.BOX_Y] -= 0.05;
                elcol.dimensions[elcol.BOX_Z] -= 0.05;
                el.second.operation = el.second.ADD;
		el.second.primitives[0] = elcol;
		ROS_ERROR("DIMS:%f",el.second.primitives[0].dimensions[elcol.BOX_X]);
                colls.push_back(el.second);
            }
            planningSceneInterface.applyCollisionObjects(colls);
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
