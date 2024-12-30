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
#include "assignment2/map_waypoints.h"
#include "assignment2/apriltag_detect.h"
#include "assignment2/WaypointMoveAction.h"
#include "assignment2/ApriltagSearchAction.h"
#include "assignment2/ObjectMoveAction.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>


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
    
    public:
        ArmMovementServer(std::string name):as(nh_, name, boost::bind(&ArmMovementServer::movementOnGoal, this, _1), false),
            action_name(name){as.start();}
        
        void movementOnGoal(const assignment2::ObjectMoveGoal::ConstPtr &goal){

            moveit::planning_interface::MoveGroupInterface moveGroup("arm_torso");
            moveGroup.setPoseReferenceFrame("base_footprint");
            moveGroup.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;   
        }

    private:

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
            float pathCartesian = moveGroup.computeCartesianPath(std::vector<geometry_msgs::Pose>{tgt}, 0.03, path);
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

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();


    return 0;
}