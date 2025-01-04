#include <ros/ros.h>
#include <map>
#include <numeric> 
#include <string> 
#include <tf2_ros/transform_listener.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include "assignment2/apriltag_detect.h"
#include "movement.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "assignment2/ObjectMoveAction.h"

struct apriltag_str{
    float x;
    float y;
    float z;
    float yaw;
    int id;
    int dock;
};

float X_STEP = 0.15;

void put_down_routine(std::vector<apriltag_str> tags, apriltag_str table_tag, int& put_objs, float m, float q, Movement& mov) {
    for (const auto& tag:tags) {
        actionlib::SimpleActionClient<assignment2::ObjectMoveAction> ac("move_object", true);
        ROS_INFO("Waiting for move_object server to start.");
        ac.waitForServer();

        // PICK UP
        ROS_INFO("Picking up object with AprilTag %u at x=%f y=%f from dock %u.", tag.id, tag.x, tag.y, tag.dock);
        mov.goAround(tag.dock);

        assignment2::ObjectMoveGoal goal_pick;
        goal_pick.pick = true;
        goal_pick.tgt_id = tag.id;
        goal_pick.tgt_pose.position.x =  tag.x;
        goal_pick.tgt_pose.position.y =  tag.y;
        goal_pick.tgt_pose.position.z =  tag.z;

        tf2::Quaternion tgt_yaw_qt;
        tgt_yaw_qt.setRPY(0,0,tag.yaw);

        goal_pick.tgt_pose.orientation.w = tgt_yaw_qt.w();
        goal_pick.tgt_pose.orientation.x = tgt_yaw_qt.x();
        goal_pick.tgt_pose.orientation.y = tgt_yaw_qt.y();
        goal_pick.tgt_pose.orientation.z = tgt_yaw_qt.z();

        ROS_INFO("Sending goal to move_object server.");
        ac.sendGoal(goal_pick);
        bool finished_before_timeout = ac.waitForResult(ros::Duration(45.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Action succeeded!");
            } else {
                ROS_WARN("Action did not succeed. State: %s", state.toString().c_str());
            }
        }
        else
            ROS_WARN("Action did not finish before timeout.");

        float put_down_y = table_tag.y + ((put_objs + 1) * X_STEP);
        float put_down_x = table_tag.x - ((put_objs + 1) * X_STEP) * m + q;

        float minDist = std::numeric_limits<float>::max(); 
        int closestDock = -1;       
        for (int dock = 1; dock <= 3; ++dock) {
            float dist = mov.dock_dist(put_down_x, put_down_y, dock);
            if (dist < minDist) {
                minDist = dist;
                closestDock = dock;
            }
        }

        //PLACE DOWN
        ROS_INFO("Placing down object with AprilTag %u in x=%f y=%f from dock %u", tag.id, put_down_x, put_down_y, closestDock);
        mov.goAround(closestDock);
        
        assignment2::ObjectMoveGoal goal_place;
        goal_place.pick = false;
        goal_place.tgt_id = tag.id;
        goal_place.tgt_pose.position.x =  put_down_x;
        goal_place.tgt_pose.position.y =  put_down_y;
        goal_place.tgt_pose.position.z =  tag.z;
        goal_place.tgt_pose.orientation.w = 0; 
        ROS_INFO("Sending goal to move_object server.");
        ac.sendGoal(goal_place);
        finished_before_timeout = ac.waitForResult(ros::Duration(45.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Action succeeded!");
            } else {
                ROS_WARN("Action did not succeed. State: %s", state.toString().c_str());
            }
        }
        else
            ROS_WARN("Action did not finish before timeout.");
	    put_objs++;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_A");
    ros::NodeHandle n;

    // Service client to obtain the parameters of the line on which the objects shall be placed
    ros::ServiceClient coeffs_client = n.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    tiago_iaslab_simulation::Coeffs srv_c;
    srv_c.request.ready = true;

    // Request the coeffs
    std::vector<float> coeffs;
    if(coeffs_client.call(srv_c)) {
        ROS_INFO("straight_line_srv call successful.");
        coeffs = srv_c.response.coeffs;
    }
    else {
        ROS_ERROR("Failed to call service straight_line_srv.");
        return 1;
    }

    ROS_INFO("Line parameters, m:%f q:%f", coeffs[0], coeffs[1]);

    ros::Duration wait_time(2.0);

    ROS_INFO("Initializing Movement object. Robot takes position at dock 2.");
    Movement mov;
    wait_time.sleep();
    ros::ServiceClient ad_client = n.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");

    std::vector<apriltag_str> dock4, dock5, dock6;
    std::map<int, apriltag_str> tags;
    apriltag_str table_tag{0,0,0,0,-1,-1};

    for(int i=3; i<=6; i++) {
        ROS_INFO("Moving to dock %u.", i);
        mov.goAround(i);
        wait_time.sleep();
        assignment2::apriltag_detect ad_srv;
        if(ad_client.call(ad_srv)) {
            ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
            std::vector<int> ids = ad_srv.response.ids;
            std::vector<float> x = ad_srv.response.x;
            std::vector<float> y = ad_srv.response.y;
            std::vector<float> z = ad_srv.response.z;
            std::vector<float> yaw = ad_srv.response.yaw;
            ROS_INFO("Detections IDs: total=%lu, content=[%s]", 
                    ids.size(),
                    std::accumulate(ids.begin(), ids.end(), std::string(),
                                    [](const std::string &a, int b) {
                                        return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b);
                                    }).c_str());
            for(int j=0; j<ids.size(); j++) {
                apriltag_str tmp{x[j], y[j], z[j], yaw[j], ids[j], i};
                if(ids[j]==10){
                    if(table_tag.dock==-1)
                        table_tag = tmp;
                    continue;
                }
                auto it = tags.find(ids[j]);
                if(it != tags.end()) {
                    if(mov.dock_dist(x[j], y[j], i) < mov.dock_dist(it->second.x, it->second.y, it->second.dock)) {
                        ROS_INFO("Updating AprilTag %u: from dock %u to dock %u", ids[j], it->second.dock, i);
                        tags[ids[j]] = tmp;
                    }
                }
                else {
                    ROS_INFO("Adding new AprilTag %u at dock %u (x=%.2f, y=%.2f)", ids[j], i, x[j], y[j]);
                    tags[ids[j]] = tmp;
                }
            }
        }  
    }

    ROS_INFO("Object detection results:");
    ROS_INFO("Table (AprilTag 10) found at x=%f y=%f from dock %u", table_tag.x, table_tag.y, table_tag.dock);
    for(auto t : tags) 
        ROS_INFO("AprilTag %u detected at x=%f y=%f from dock %u", t.second.id, t.second.x, t.second.y, t.second.dock);

    for(auto tag : tags){
	    if(tag.second.dock==4)
		    dock4.push_back(tag.second);
	    else if(tag.second.dock==5)
		    dock5.push_back(tag.second);
	    else if(tag.second.dock==6)
		    dock6.push_back(tag.second);
    }
    assignment2::apriltag_detect ad_srv;
    ad_srv.request.create_collisions = true;
    ROS_INFO("Creating collision objects from detections.");
    if(ad_client.call(ad_srv))
        ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");

    ROS_INFO("Starting the object placing routine.");
    int put_objs = 0;
    put_down_routine(dock4, table_tag, put_objs, coeffs[0], coeffs[1], mov);

    // for(int i=5; i>=1; i--) {
    //     ROS_INFO("Moving to dock %u.", i);
    //     mov.goAround(i);
    //     wait_time.sleep();
    // }

    return 0;
}