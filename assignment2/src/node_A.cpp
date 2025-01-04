#include <ros/ros.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include "assignment2/object_detect.h"
#include "assignment2/apriltag_detect.h"
#include "movement.h"
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
	ROS_INFO("moving to: %d", tag.dock);
        mov.goAround(tag.dock);
        actionlib::SimpleActionClient<assignment2::ObjectMoveAction> ac("move_object", true);
        ROS_INFO("Waiting for Movemente_Handler server to start...");
        ac.waitForServer();

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

        
        ac.sendGoal(goal_pick);
        ac.waitForResult(ros::Duration(30.0));
        ROS_INFO("Movemente_Handler server started. Now looking for the closest waypoint.");
        //pickupobject(cur_obj);

        // CHECK THIS, IS THE SAME REF. FRAME ??
        float put_down_y = table_tag.y + ((put_objs + 1) * X_STEP);
        float put_down_x = table_tag.x - ((put_objs + 1) * X_STEP) * m + q;
        float dist1 = mov.dock_dist(put_down_x, put_down_y, 1);
        float dist2 = mov.dock_dist(put_down_x, put_down_y, 2);
        float dist3 = mov.dock_dist(put_down_x, put_down_y, 3);
        if(dist1 < dist2 && dist1 < dist3) 
            mov.goAround(1);
        else if(dist2 < dist3) 
            mov.goAround(2);
        else 
            mov.goAround(3);

        //put down
        assignment2::ObjectMoveGoal goal_place;
        goal_place.pick = false;
        goal_place.tgt_id = tag.id;
        goal_place.tgt_pose.position.x =  put_down_x;
        goal_place.tgt_pose.position.y =  put_down_y;
        goal_place.tgt_pose.position.z =  tag.z;
        goal_place.tgt_pose.orientation.w = 0; 
        ac.sendGoal(goal_place);
        ac.waitForResult(ros::Duration(30.0));
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

    ROS_INFO("Initializing Movement object. Robot takes position at dock 1.");
    Movement mov;
    wait_time.sleep();
    ros::ServiceClient ad_client = n.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");
    assignment2::apriltag_detect ad_srv;

    std::vector<apriltag_str> dock4, dock5, dock6;
    std::map<int, apriltag_str> tags;
    apriltag_str table_tag{0,0,0,0,-1,-1};

    for(int i=3; i<=6; i++) {
        ROS_INFO("Moving to dock %u.", i);
        mov.goAround(i);
        wait_time.sleep();
        if(ad_client.call(ad_srv)) {
            ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
            std::vector<int> ids = ad_srv.response.ids;
            std::vector<float> x = ad_srv.response.x;
            std::vector<float> y = ad_srv.response.y;
            std::vector<float> z = ad_srv.response.z;
            std::vector<float> yaw = ad_srv.response.yaw;
            for(int j=0; j<ids.size(); j++) {
                apriltag_str tmp{x[j], y[j], z[j], yaw[j], ids[j], i};
                if(ids[j]==10){
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
    for(auto tag : tags){
	    if(tag.second.dock==4)
		    dock4.push_back(tag.second);
	    else if(tag.second.dock==5)
		    dock4.push_back(tag.second);
	    else if(tag.second.dock==6)
		    dock4.push_back(tag.second);
    }
    int put_objs;
    put_down_routine(dock4, table_tag, put_objs, coeffs[0], coeffs[1], mov);
    ad_srv.request.create_collisions = true;
    ad_client.call(ad_srv);

    ROS_INFO("Object detection results:");
    ROS_INFO("Table (AprilTag 10) found at x=%f y=%f from dock %u", table_tag.x, table_tag.y, table_tag.dock);
    for(auto t : tags) 
        ROS_INFO("AprilTag %u detected at x=%f y=%f from dock %u", t.second.id, t.second.x, t.second.y, t.second.dock);

    // for(int i=5; i>=1; i--) {
    //     ROS_INFO("Moving to dock %u.", i);
    //     mov.goAround(i);
    //     wait_time.sleep();
    // }
return 0;
}
