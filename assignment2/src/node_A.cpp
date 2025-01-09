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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

struct apriltag_str{
    float x;
    float y;
    float z;
    float yaw;
    int id;
    int dock;
};

const float TABLE_SIDE = 0.85;
const float PADDING    = 0.1;

std::pair<float,float> compute_coord(float start_x, float start_y, int count, float m, float q){
    const int EXTRA = 2;
    float x_step = (TABLE_SIDE - PADDING - q) / ((3 + EXTRA) * m);
    return std::make_pair(start_x - ((count + 1+EXTRA) * x_step) * m - q, start_y + ((count + 1+EXTRA) * x_step));
}

void add_reference_collisions(float start_x, float start_y, float m, float q){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    for(int i = 0; i < 3; i++){
        auto coords = compute_coord(start_x, start_y, i, m, q);
        moveit_msgs::CollisionObject collision_object;
        collision_object.operation = collision_object.ADD;
        collision_object.id = "reference_obj_"+std::to_string(i);
        collision_object.header.frame_id = "/map"; // Replace with the appropriate frame ID
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X]= 0.1; // Dimensions: X, Y, Z
        primitive.dimensions[primitive.BOX_Y]= 0.1; // Dimensions: X, Y, Z
        primitive.dimensions[primitive.BOX_Z]= 0.5; // Dimensions: X, Y, Z
        geometry_msgs::Pose box_pose;
        box_pose.position.x = coords.first;
        box_pose.position.y = coords.second;
        box_pose.position.z = 3.0;
        box_pose.orientation.w = 1.0;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_objects.push_back(collision_object);
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);

}

void add_collision_objs(std::map<int, apriltag_str> tags, ros::ServiceClient& ad_client) {
    assignment2::apriltag_detect ad_srv_coll;
    ad_srv_coll.request.create_collisions = true;
    std::vector<int> ids;
    std::vector<float> x, y, z, yaw;
    for(auto tag : tags) {
        ids.push_back(tag.second.id);
        x.push_back(tag.second.x);
        y.push_back(tag.second.y);
        z.push_back(tag.second.z);
        yaw.push_back(tag.second.yaw);
    }
    ad_srv_coll.request.ids = ids;
    ad_srv_coll.request.x = x;
    ad_srv_coll.request.y = y;
    ad_srv_coll.request.z = z;
    ad_srv_coll.request.yaw = yaw;
    ROS_INFO("Creating collision objects from detections.");
    if(ad_client.call(ad_srv_coll))
        ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
}

void remove_collision_obj(int id_picked) {
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    std::map<std::string, moveit_msgs::CollisionObject> colls = planningSceneInterface.getObjects();
    if(id_picked!=-1) {
        std::string object_id = "box_april_"+std::to_string(id_picked);
        colls.erase(object_id);
    }
    std::vector<moveit_msgs::CollisionObject> to_remove;
    for(auto it=colls.begin(); it!=colls.end(); it++){
        to_remove.push_back(it->second);
        it->second.operation=it->second.REMOVE;
    }  
    planningSceneInterface.applyCollisionObjects(to_remove);
}

void put_down_routine(std::map<int, apriltag_str>& tags, int to_pick, apriltag_str table_tag, int& put_objs, float m, float q, Movement& mov, ros::ServiceClient& ad_client) {
    actionlib::SimpleActionClient<assignment2::ObjectMoveAction> ac("move_object", true);
    ROS_INFO("Waiting for move_object server to start.");
    ac.waitForServer();

    apriltag_str tag = tags[to_pick];

    // PICK UP
    ROS_INFO("Picking up object with AprilTag %u at x=%f y=%f from dock %u.", tag.id, tag.x, tag.y, tag.dock);
    mov.goAround(tag.dock);

    add_collision_objs(tags, ad_client);

    assignment2::ObjectMoveGoal goal_pick;
    goal_pick.pick = true;
    goal_pick.tgt_id = tag.id;
    goal_pick.tgt_pose.position.x =  tag.x;
    goal_pick.tgt_pose.position.y =  tag.y;
    goal_pick.tgt_pose.position.z =  tag.z;

    tf2::Quaternion qt;
    if(tag.id==7 || tag.id==8 || tag.id==9)
        qt.setRPY(0, M_PI_2, tag.yaw);
    else 
        qt.setRPY(0, M_PI_2, tag.yaw - M_PI_2);

    goal_pick.tgt_pose.orientation.w = qt.w();
    goal_pick.tgt_pose.orientation.x = qt.x();
    goal_pick.tgt_pose.orientation.y = qt.y();
    goal_pick.tgt_pose.orientation.z = qt.z();

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

    remove_collision_obj(tag.id);
    tags.erase(tag.id);
    
    std::pair<float, float> coords = compute_coord(table_tag.x, table_tag.y, put_objs, m, q);
    float put_down_y = coords.second;
    float put_down_x = coords.first;

    int closestDock = mov.closest_dock(put_down_x, put_down_y);

    //PLACE DOWN
    ROS_INFO("Placing down object with AprilTag %u in x=%f y=%f from dock %u", tag.id, put_down_x, put_down_y, closestDock);
    mov.goAround(closestDock);

    add_collision_objs(tags, ad_client);
    
    assignment2::ObjectMoveGoal goal_place;
    goal_place.pick = false;
    goal_place.tgt_id = tag.id;
    goal_place.tgt_pose.position.x =  put_down_x;
    goal_place.tgt_pose.position.y =  put_down_y;
    goal_place.tgt_pose.position.z =  tag.z;
    qt.setRPY(0, M_PI_2, 0);
    goal_place.tgt_pose.orientation.w = qt.w();
    goal_place.tgt_pose.orientation.x = qt.x();
    goal_place.tgt_pose.orientation.y = qt.y();
    goal_place.tgt_pose.orientation.z = qt.z();
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

    remove_collision_obj(-1);
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
    // wait_time.sleep();
    ros::ServiceClient ad_client = n.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");

    std::vector<apriltag_str> dock4, dock5, dock6;
    std::map<int, apriltag_str> tags;
    apriltag_str table_tag{0,0,0,0,-1,-1};
    apriltag_str to_move{0,0,0,0,-1,-1};
    int put_objs = 0;

    for(int i=3; i<=6; i++) {
        std::vector<double> turns;
        if(i==3) turns = {0.0};
        else turns = {0.0, -M_PI_4/3, 2*M_PI_4/3};

        // REMOVE THIS
        if(i==4 || i==6) continue;

        do {
            ROS_INFO("Moving to dock %u.", i);
            mov.goAround(i);
            for(auto turn: turns) {
                mov.spin(turn);
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
                                int closestDock = mov.closest_dock(x[j], y[j]);
                                tmp.dock = closestDock;
                                ROS_INFO("Updating AprilTag %u: from dock %u to dock %u", ids[j], it->second.dock, closestDock);
                                tags[ids[j]] = tmp;
                            }
                        }
                        else {
                            int closestDock = mov.closest_dock(x[j], y[j]);
                            tmp.dock = closestDock;
                            ROS_INFO("Adding new AprilTag %u at dock %u (x=%.2f, y=%.2f)", ids[j], closestDock, x[j], y[j]);
                            tags[ids[j]] = tmp;
                        }
                    }
                }
            }

            if(i==3) continue;

            ROS_INFO("Object detection results:");
            ROS_INFO("Table (AprilTag 10) found at x=%f y=%f from dock %u", table_tag.x, table_tag.y, table_tag.dock);
            for(auto t : tags) 
                ROS_INFO("AprilTag %u detected at x=%f y=%f from dock %u", t.second.id, t.second.x, t.second.y, t.second.dock);

            mov.fix_pos();

            // add_reference_collisions(table_tag.x, table_tag.y, coeffs[0], coeffs[1]);

            to_move.id = -1;
            for(auto tag : tags)
                if(to_move.id == -1 && tag.second.dock==i)
                    to_move = tag.second;
            if(to_move.id != -1) {
                ROS_INFO("Starting the object placing routine.");
                put_down_routine(tags, to_move.id, table_tag, put_objs, coeffs[0], coeffs[1], mov, ad_client);
            }
            else 
                ROS_INFO("No object left to move from dock %u.", i);
        }  
        while(to_move.id != -1);
    }

    // for(int i=5; i>=1; i--) {
    //     ROS_INFO("Moving to dock %u.", i);
    //     mov.goAround(i);
    //     wait_time.sleep();
    // }

    return 0;
}
