#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include "assignment2/object_detect.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>


// CONSTANTS
float TABLE_1_X = 7.82;
float TABLE_1_Y = -1.96;
float TABLE_2_X = 7.82;
float TABLE_2_Y = -3.01;
float DIST      = 1.0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const geometry_msgs::Quaternion POS_X_ORIENTATION = [] {
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;  // 0° (Positive X)
    return q;
}();

const geometry_msgs::Quaternion POS_Y_ORIENTATION = [] {
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = 0.707; q.w = 0.707;  // 90° (Positive Y)
    return q;
}();

const geometry_msgs::Quaternion NEG_X_ORIENTATION = [] {
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = 1.0; q.w = 0.0;  // 180° (Negative X)
    return q;
}();

const geometry_msgs::Quaternion NEG_Y_ORIENTATION = [] {
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = -0.707; q.w = 0.707;  // -90° (Negative Y)
    return q;
}();

int curpos= 1;
std::vector<std::pair<float, float>> docks, corns;


void sendGoalToMoveBase(double x, double y, const geometry_msgs::Quaternion& orientation) {

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;  // Flat ground
    goal.target_pose.pose.orientation = orientation;

    ROS_INFO("Sending goal to MoveBase: x=%.2f, y=%.2f, orientation=[%.2f, %.2f, %.2f, %.2f]",
             x, y, orientation.x, orientation.y, orientation.z, orientation.w);
    // Assuming you have already created and initialized a MoveBaseClient instance called `ac`
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached successfully!");
    } else {
        ROS_WARN("Failed to reach the goal.");
    }
}
void go_clockwise(int target_pos){
    //main objective:survive
    geometry_msgs::Quaternion escape_orientation;
    if(curpos==2){
        escape_orientation = POS_X_ORIENTATION;
    }
    if(curpos==3 || curpos==4){
        escape_orientation = NEG_Y_ORIENTATION;
    }
    if(curpos==6 || curpos==1){
        escape_orientation = POS_Y_ORIENTATION;
    }
    sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, escape_orientation);

    if(curpos==1&&target_pos>curpos){
        sendGoalToMoveBase(corns[0].first, corns[0].second, POS_Y_ORIENTATION);
        sendGoalToMoveBase(corns[0].first, corns[0].second, POS_X_ORIENTATION);
        curpos++;
        sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, POS_X_ORIENTATION);
        if(target_pos==2){
            sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_Y_ORIENTATION);
            return;
        }
    }
    if(curpos==2&&target_pos>curpos){
        sendGoalToMoveBase(corns[1].first, corns[1].second, POS_X_ORIENTATION);
        sendGoalToMoveBase(corns[1].first, corns[1].second, NEG_Y_ORIENTATION);
        curpos++;
        sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==3){
            sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(curpos==3&&target_pos>curpos){
        curpos++;
        sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_Y_ORIENTATION);
        if(target_pos==4){
            sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_X_ORIENTATION);
            return;
        }
    }
    if(curpos==4&&target_pos>curpos){
        sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_Y_ORIENTATION);
        sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_X_ORIENTATION);
        curpos++;
        sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, NEG_X_ORIENTATION);
        if(target_pos==5){
            sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, POS_Y_ORIENTATION);
            return;
        }
    }
    if(curpos==5&&target_pos>curpos){
        sendGoalToMoveBase(corns[3].first, corns[3].second, NEG_X_ORIENTATION);
        sendGoalToMoveBase(corns[3].first, corns[3].second, POS_Y_ORIENTATION);
        curpos++;
        sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, POS_Y_ORIENTATION);
        if(target_pos==6){
            sendGoalToMoveBase(docks[curpos-1].first, docks[curpos-1].second, POS_X_ORIENTATION);
            return;
        }
    }
}
void goAround(int target_pos){
    if(target_pos >curpos){
        go_clockwise(target_pos);
    }
}

bool detection_routine(assignment2::object_detect::Request &req, assignment2::object_detect::Response &res)
{
    if(req.ready == false) {
        ROS_WARN("Detection request is set to false.");
        return false;
    }

    ROS_INFO("Service called. Detection routine starts.");

    std::vector<std::pair<float, float>> docks_offsets = {
        std::make_pair(-DIST, 0), 
        std::make_pair(0, -DIST), 
        std::make_pair(DIST, 0)
    };
    std::vector<std::pair<float, float>> corns_offsets = {
        std::make_pair(-DIST, DIST),
        std::make_pair(DIST, DIST) 
    };

    for(auto off : docks_offsets) 
        docks.push_back(std::make_pair(TABLE_1_X + off.first, TABLE_1_Y - off.second));
    for(auto off : docks_offsets) 
        docks.push_back(std::make_pair(TABLE_2_X - off.first, TABLE_2_Y + off.second));
    for(auto off : corns_offsets) 
        corns.push_back(std::make_pair(TABLE_1_X + off.first, TABLE_1_Y + off.second));
    for(auto off : corns_offsets) 
        corns.push_back(std::make_pair(TABLE_2_X - off.first, TABLE_2_Y - off.second));

    ROS_INFO("Docking positions:");
    for(auto p : docks)
        ROS_INFO("x=%f y=%f", p.first, p.second);
    ROS_INFO("Corner positions:");
    for(auto p : corns)
        ROS_INFO("x=%f y=%f", p.first, p.second);

    std::vector<float> objs_x, objs_y, docks_x, docks_y;
    std::vector<int> ids;

    sendGoalToMoveBase(docks[0].first, 0.0, POS_X_ORIENTATION);
    goAround(6);
    //// Intermediate point
    //sendGoalToMoveBase(docks[0].first, 0.0, NEG_Y_ORIENTATION);

    //// First table side
    //sendGoalToMoveBase(docks[0].first, docks[0].second, POS_X_ORIENTATION);
    //// DETECTION HERE
    //sendGoalToMoveBase(docks[0].first, docks[0].second, NEG_Y_ORIENTATION);

    //// First corner
    //sendGoalToMoveBase(corns[0].first, corns[0].second, POS_X_ORIENTATION);   

    //// Second table side
    //sendGoalToMoveBase(docks[1].first, docks[1].second, POS_Y_ORIENTATION);
    //// DETECTION HERE
    //sendGoalToMoveBase(docks[1].first, docks[1].second, POS_X_ORIENTATION); 

    //// Second corner
    //sendGoalToMoveBase(corns[1].first, corns[1].second, POS_Y_ORIENTATION);   

    //// Third table side
    //sendGoalToMoveBase(docks[2].first, docks[2].second, NEG_X_ORIENTATION);
    //// DETECTION HERE

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
