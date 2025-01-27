#include <ros/ros.h>
#include <map>
#include <cstdlib>
#include <ctime>
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

// Struct for AprilTags info
struct apriltag_str{
    float x;
    float y;
    float z;
    float yaw;
    int id;
    int color;
    int dock;
};

// Class for Node A operations handling
class Node_A {

    private:
        // Table properties
        const float TABLE_SIDE = 0.91;
        const float PADDING    = 0.07;

        ros::NodeHandle nh_;
        Movement mov;
        ros::ServiceClient ad_client;
        float m;
        float q;
        int color_to_pick;
        int placed_objs;
        int placed_last;
        std::map<int, apriltag_str> tags_pick;
        std::map<int, apriltag_str> tags_place;

    public:
        // Basic constructor, launches setup routine for the pick_and_place one 
        Node_A() : mov(nh_) {
            // Service client to obtain the parameters of the line on which the objects shall be placed
            ros::ServiceClient coeffs_client = nh_.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
            tiago_iaslab_simulation::Coeffs srv_c;
            srv_c.request.ready = true;
            // Request the coeffs
            std::vector<float> coeffs;
            if(coeffs_client.call(srv_c)) {
                ROS_INFO("straight_line_srv call successful.");
                coeffs = srv_c.response.coeffs;
                m = coeffs[0];
                q = coeffs[1];
                ROS_INFO("Line parameters, m:%f q:%f", coeffs[0], coeffs[1]);
            }
            else
                ROS_ERROR("Failed to call service straight_line_srv.");

            // Randomly choose the color of the objects that shall be picked
            srand(time(0));
            color_to_pick = rand()%3;
            std::map<int, std::string> colorMap = {{0, "Blue"}, {1, "Red"}, {2, "Green"}};
            ROS_INFO("Object color choosen to be picked: %s", colorMap[color_to_pick].c_str());

            // Initialize placed objects counter
            placed_objs = 0;
            placed_last = -1;

            // Initializing detection service client
            ad_client = nh_.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");
        }

        // Compute the object placing coordinates, result depends on the numebr of objects placed so far 
        std::pair<float,float> compute_coord(float start_x, float start_y, float yaw) {
            const float OBJ_DIST = 0.12;
            const float TABLE_DIST = 0.08;
            float OBJ_DIST_X = OBJ_DIST * cos(atan(m));
            float TABLE_DIST_X = TABLE_DIST * cos(atan(m));
            float x_max = std::min((TABLE_SIDE-PADDING-q)/m, TABLE_SIDE-PADDING);
            float tgt_x = x_max - TABLE_DIST_X - OBJ_DIST_X*(3-placed_objs-1);  
            float tgt_y = tgt_x * m + q;
            return std::make_pair(start_x +cos(yaw)*(tgt_x) - sin(yaw)*(tgt_y) , start_y + sin(yaw)*(tgt_x) + cos(yaw)*(tgt_y));
        }

        // Refine object placing coordinates using detection on last placed object
        bool refine_coord(std::pair<float, float>& coord, float yaw) {
            ROS_INFO("Refining placing coordinates...");
            const float OBJ_DIST = 0.12;
            float OBJ_DIST_X = OBJ_DIST * cos(atan(m));
            std::vector<int> ids, colors;
            std::vector<float> x, y, z, yaws;
            assignment2::apriltag_detect ad_srv;
            float tgt_x, tgt_y;
            if(ad_client.call(ad_srv)) {
                ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
                load_detections(ad_srv, ids, colors, x, y, z, yaws);
                for(int i=0; i<ids.size(); i++) {
                    if(ids[i]==placed_last) {
                        ROS_INFO("Placing coordinates refined.");
                        tgt_x = OBJ_DIST_X;
                        tgt_y = OBJ_DIST_X*m;
                        coord = std::make_pair(x[i]+cos(yaw)*(tgt_x) - sin(yaw)*(tgt_y) , y[i] + sin(yaw)*(tgt_x) + cos(yaw)*(tgt_y));
			            return true;
                    }
                }
            }
            ROS_INFO("Previous placed object not found, placing coordinates not refined.");
	        return false;
        }

        // Create collision objects from given data calling Node B service
        void add_collision_objs(std::map<int, apriltag_str> tags) {
            assignment2::apriltag_detect ad_srv_coll;
            ad_srv_coll.request.create_collisions = true;
            std::vector<int> ids, colors;
            std::vector<float> x, y, z, yaw;
            for(auto tag : tags) {
                ids.push_back(tag.second.id);
                colors.push_back(tag.second.color);
                x.push_back(tag.second.x);
                y.push_back(tag.second.y);
                z.push_back(tag.second.z);
                yaw.push_back(tag.second.yaw);
            }
            ad_srv_coll.request.id = ids;
            ad_srv_coll.request.color = colors;
            ad_srv_coll.request.x = x;
            ad_srv_coll.request.y = y;
            ad_srv_coll.request.z = z;
            ad_srv_coll.request.yaw = yaw;
            ad_srv_coll.request.table_1_x = mov.TABLE_1_X;
            ad_srv_coll.request.table_1_y = mov.TABLE_1_Y;
            ad_srv_coll.request.table_2_x = mov.TABLE_2_X;
            ad_srv_coll.request.table_2_y = mov.TABLE_2_Y;
            ROS_INFO("Creating collision objects from detections.");
            if(ad_client.call(ad_srv_coll))
                ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
        }
        
        // Pick up object with given id and place it in the next placing position
        void pick_place_routine(int to_pick, apriltag_str table_tag, double turn) {
            actionlib::SimpleActionClient<assignment2::ObjectMoveAction> ac("move_object", true);
            ROS_INFO("Waiting for move_object server to start.");
            ac.waitForServer();

            // Info on the object to pick
            apriltag_str tag = tags_pick[to_pick];

            // PICK UP PHASE
            ROS_INFO("Picking up object with AprilTag %u at x=%f y=%f from dock %u.", tag.id, tag.x, tag.y, tag.dock);

            // Update objects info and create collision objects
            for(auto& tag_placed : tags_place) 
                tags_pick[tag_placed.first] = tag_placed.second;
            add_collision_objs(tags_pick);
            
            // Setup goal to pick up object
            assignment2::ObjectMoveGoal goal_pick;
            goal_pick.pick = true;
            goal_pick.tgt_id = tag.id;
            goal_pick.tgt_pose.position.x =  tag.x;
            goal_pick.tgt_pose.position.y =  tag.y;
            goal_pick.tgt_pose.position.z =  tag.z;
            tf2::Quaternion qt;
            if(tag.color == 2) // Different (better) gripper orientation for green objects
                qt.setRPY(0, M_PI_2, tag.yaw);
            else 
                qt.setRPY(0, M_PI_2, tag.yaw - M_PI_2);
            goal_pick.tgt_pose.orientation.w = qt.w();
            goal_pick.tgt_pose.orientation.x = qt.x();
            goal_pick.tgt_pose.orientation.y = qt.y();
            goal_pick.tgt_pose.orientation.z = qt.z();

            // Send picking goal 
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

            // remove_collision_obj(tag.id);
            tags_pick.erase(tag.id);
            
            // Compute next ideal placing coordinates
            std::pair<float, float> coords = compute_coord(table_tag.x, table_tag.y, table_tag.yaw);
            float put_down_x = coords.first;
            float put_down_y = coords.second;

            //PLACE DOWN PHASE
            int closestDock = mov.closest_dock(put_down_x, put_down_y);
            ROS_INFO("Placing down object with AprilTag %u from dock %u", tag.id, closestDock);

            // Move to optimal placing dock
            mov.spin(-turn);
            mov.goAround(closestDock);

            // Obtain real placing coordinates if possible
	        std::vector<double> turns = {0.0, -M_PI_4/3, 2*M_PI_4/3};
            if(placed_last!=-1){
                for(auto turn_place: turns) {
                    mov.spin(turn_place);
                    if(refine_coord(coords, table_tag.yaw)) {
                        put_down_x = coords.first;
                        put_down_y = coords.second;
                        break;
                    }
                }
            }
            ROS_INFO("Placing coordinates: x=%f y=%f", put_down_x, put_down_y);

            // Reload collision objects
            add_collision_objs(tags_pick);
            
            // Setup and send goal for object placing
            assignment2::ObjectMoveGoal goal_place;
            goal_place.pick = false;
            goal_place.tgt_id = tag.id;
            goal_place.tgt_pose.position.x =  put_down_x;
            goal_place.tgt_pose.position.y =  put_down_y;
            goal_place.tgt_pose.position.z =  tag.z;
            qt.setRPY(0, M_PI_2, M_PI_2);
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

            // Update data with latest placing
            placed_objs++;
            tag.x = put_down_x;
            tag.y = put_down_y;
            tag.yaw = 0.0;
            tag.dock = closestDock;
            tags_place[tag.id] = tag;
            placed_last = tag.id;
        }
        // Load data structures with detection info
        void load_detections(assignment2::apriltag_detect& ad_srv, std::vector<int>& ids, std::vector<int>& colors, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, std::vector<float>& yaw) {
            ids = ad_srv.response.id;
            colors = ad_srv.response.color;
            x = ad_srv.response.x;
            y = ad_srv.response.y;
            z = ad_srv.response.z;
            yaw = ad_srv.response.yaw;
            // ROS_INFO("Detections IDs: total=%lu, content=[%s]", 
            //     ids.size(),
            //     std::accumulate(ids.begin(), ids.end(), std::string(),
            //     [](const std::string &a, int b) {
            //         return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b);
            //     }).c_str());
        }

        // Launch the full detect, pick and place routine for all objects of the chosen color
        void launch() {
            apriltag_str table_tag{0,0,0,0,-1,-1,-1};
            apriltag_str to_move{0,0,0,0,-1,-1,-1};
            std::vector<int> ids, colors;
            std::vector<float> x, y, z, yaw;
            std::map<int, std::string> colorMap = {{0, "Blue"}, {1, "Red"}, {2, "Green"}};
            // Perform the routine from all 6 docking stations
            for(int i=1; i<=6; i++) {
                std::vector<double> turns = {0.0, -M_PI_4/3, 2*M_PI_4/3};
                // Loop on the routine until no object that meets the criteria is found and left to be moved
                do {
                    ROS_INFO("Moving to dock %u.", i);
                    // At each dock, change slightly the orientation to ensure no AprilTag is missed
                    for(auto turn: turns) {
                        mov.goAround(i);
                        mov.spin(turn);
                        // Perform AprilTag detection and load data structures
                        assignment2::apriltag_detect ad_srv;
                        ROS_INFO("Sending request to apriltags_detected_service.");
                        if(ad_client.call(ad_srv)) {
                            ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
                            load_detections(ad_srv, ids, colors, x, y, z, yaw);
                            tags_pick.clear();
                            for(int j=0; j<ids.size(); j++) {
                                apriltag_str tmp{x[j], y[j], z[j], yaw[j], ids[j], colors[j], i};
                                // Save separately info on the table tag
                                if(ids[j]==10){
                                    if(table_tag.dock==-1)
                                        table_tag = tmp;
                                    continue;
                                }
                                // Associate each detection with the dock closest to it
                                int closestDock = mov.closest_dock(x[j], y[j]);
                                tmp.dock = closestDock;
                                tags_pick[ids[j]] = tmp;
                                // ROS_INFO("Adding AprilTag %u to dock %u (x=%.2f, y=%.2f)", ids[j], closestDock, x[j], y[j]);
                            }
                        }
                        // Detection results printing
                        if(!ids.empty()) {
                            ROS_INFO("Object detection results:");
                            if(table_tag.dock!=-1)
                                ROS_INFO("Table (AprilTag 10) found at x=%f y=%f from dock %u.", table_tag.x, table_tag.y, table_tag.dock);
                            for(auto t : tags_pick) 
                                ROS_INFO("AprilTag %u with color %u detected at x=%f y=%f (closest_dock=%u).", t.second.id, t.second.color, t.second.x, t.second.y, t.second.dock);
                        }
                        else    
                            ROS_INFO("No AprilTags found.");
                        // Check if a detection meets the conditions, if there i start the pick and place routine for it
                        to_move.id = -1;
                        for(auto tag : tags_pick)
                            if(to_move.id == -1 && tag.second.dock==i && tag.second.color==color_to_pick)
                                to_move = tag.second;
                        if(to_move.id != -1) {
                            ROS_INFO("Starting the object placing routine.");
                            pick_place_routine(to_move.id, table_tag, turn);
                            if(placed_objs==3) return;
                        }
                    }
                }  
                while(to_move.id != -1);
                ROS_INFO("No object with color %s left to move from dock %u.", colorMap[color_to_pick].c_str(), i);
            }
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_A");

    // Set up the system and launch the detect, pick and place routine
    Node_A node;
    node.launch();

    return 0;
}
