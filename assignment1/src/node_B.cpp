#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <cstdint>
#include <thread>        
#include <chrono>
#include "assignment1/map_waypoints.h"
#include "assignment1/apriltag_detect.h"
#include "assignment1/WaypointMoveAction.h"
#include "assignment1/ApriltagSearchAction.h"

const float MAX_CORRIDOR_X = 5.66;

// Typedefs for better readability
typedef actionlib::SimpleActionClient<assignment1::WaypointMoveAction> WaypointMoveClient;
typedef assignment1::WaypointMoveFeedbackConstPtr FeedbackPtr;
typedef assignment1::WaypointMoveResultConstPtr ResultPtr;

class Coordinator {

    protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment1::ApriltagSearchAction> as_;
    std::string action_name_;
    assignment1::ApriltagSearchFeedback feedback_;
    assignment1::ApriltagSearchResult result_;
    bool custom_mcl_flag;
    
    public:

    Coordinator(std::string name, bool flag) : as_(nh_, name, boost::bind(&Coordinator::executeCB, this, _1), false), action_name_(name) 
    {
        custom_mcl_flag = flag;
        as_.start();
    }

    ~Coordinator(void){}

    void executeCB(const assignment1::ApriltagSearchGoalConstPtr &goal)
    {
        std::vector<int> ids = goal-> ids;
        std::map<int, bool> map_atfound;
        std::map<int, std::pair<float, float>> map_atcoord;
        for(int id : ids) {
            map_atfound[id] = false;                 
            map_atcoord[id] = std::make_pair(0.0f, 0.0f); 
        }

        feedback_.status = {"Robot is requesting waypoints for the navigation."};
        as_.publishFeedback(feedback_);

        // Service client to get the waypoints
        ros::ServiceClient wp_client = nh_.serviceClient<assignment1::map_waypoints>("map_waypoints_service");
        assignment1::map_waypoints srv;
        const float TILE_DIM = 2;
        srv.request.dim = TILE_DIM;

        std::vector<float> x, y;
        std::vector<std::pair<float, float>> waypoints;
        if(wp_client.call(srv)) {
            ROS_INFO("Call to map_waypoints_service: SUCCESSFUL.");
            x = srv.response.x;
            y = srv.response.y;
            for (int i=0; i<x.size(); i++) {
                if(custom_mcl_flag && x[i] <= MAX_CORRIDOR_X)
                    continue;
                waypoints.emplace_back(x[i], y[i]);
            }        
            ROS_INFO("Received %lu waypoints.", x.size());
        }
        else {
            ROS_ERROR("Call to service map_waypoints_service: FAILED.");
            return;
        }

        feedback_.status = {"Waypoints loaded, Robot starts the navigation."};
        as_.publishFeedback(feedback_);

        // CHANGE WITH THE REAL CURRENT POS
        std::pair<float,float> curr_pos = get_robot_pos();
        ROS_INFO("Initial position, x:%f y:%f", curr_pos.first, curr_pos.second);

        // Action client to send the waypoint goal
        WaypointMoveClient ac("move_to_goal", true);
        ROS_INFO("Waiting for Movemente_Handler server to start...");
        ac.waitForServer();
        ROS_INFO("Movemente_Handler server started. Now looking for the closest waypoint.");

        int tot_waypoints = waypoints.size();
        int counter = 0;
        while(waypoints.size() > 0) {
            auto closest_it = waypoints.begin();
            float min_distance = distanceSquared(curr_pos.first, curr_pos.second, closest_it->first, closest_it->second);

            for (auto it = waypoints.begin() + 1; it != waypoints.end(); ++it) {
                float dist = distanceSquared(curr_pos.first, curr_pos.second, it->first, it->second);
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_it = it;
                }
            }

            assignment1::WaypointMoveGoal wp_goal;
            wp_goal.x = (*closest_it).first;
            wp_goal.y = (*closest_it).second;
            ROS_INFO("Next Waypoint x:%f y:%f", wp_goal.x, wp_goal.y);

            feedback_.status = {"Robot resumes navigation and scanning..."};
            as_.publishFeedback(feedback_);

            // Send the goal with result and feedback callbacks, and no active callback
            ac.sendGoal(
                wp_goal,
                boost::bind(&Coordinator::resultCallback, this, _1, _2),  
                WaypointMoveClient::SimpleActiveCallback(),
                boost::bind(&Coordinator::feedbackCallback, this, _1)
            );

            bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));
            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Waypoint processing result: %s", state.toString().c_str());
            } else {
                ROS_INFO("Waypoint not processed before the timeout.");
            }

            // Update curr_pos
            curr_pos = get_robot_pos();

            // Remove the previous waypoint from the vector
            waypoints.erase(closest_it);
            
            ++counter;
            feedback_.status = {"Robot has processed a waypoint (" + std::to_string(counter) + "/" + std::to_string(tot_waypoints) + ")."};
            as_.publishFeedback(feedback_);

            ros::ServiceClient ad_client = nh_.serviceClient<assignment1::apriltag_detect>("apriltags_detected_service");
            assignment1::apriltag_detect ad_srv;

            if(ad_client.call(ad_srv)) {
                ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
                // TAGS FILTERING AND MANAGEMENT
                std::vector<int> ids = ad_srv.response.ids;
                for(int i=0; i<ids.size(); i++) {
                    auto it = map_atfound.find(ids[i]);
                    if(it != map_atfound.end()) {
                        map_atfound[ids[i]] = true;
                        map_atcoord[ids[i]] = std::make_pair(ad_srv.response.x[i], ad_srv.response.y[i]);
                    }
                }
            }
            else {
                ROS_ERROR("Call to apriltags_detected_service: FAILED.");
                return;
            }

            // FEEDBACK ON ACTION FOR HOW MANY TAGS HAVE BEEN FOUND (with ids)
            int ids_counter = 0;
            std::string ids_list1 =  "";
            std::string ids_list2 =  "";
            for(int id : ids) {
                if(map_atfound[id]==true) {
                    ids_counter++;
                    ids_list1 = ids_list1 + " " + std::to_string(id);
                } else {
                    ids_list2 = ids_list2 + " " + std::to_string(id);
                }
            }
            feedback_.status = {"Apriltags search current status: "+std::to_string(ids_counter)+"/"+std::to_string(ids.size()) ,
                                "Apriltags found:  " + ids_list1,
                                "Apriltags missing:" + ids_list2};
            as_.publishFeedback(feedback_);
        }

        ROS_INFO("All Waypoints processed, coordinator node shuts down.");
        feedback_.status = {"Robot processed every waypoint. Navigation and Scanning operations completed."};
        as_.publishFeedback(feedback_);
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

        // RESULT ON ACTION
        std::vector<float> res_x;
        std::vector<float> res_y;
        std::vector<uint8_t>  res_f;
        for(int id : ids) {
            if(map_atfound[id]==true) {
                res_x.push_back(map_atcoord[id].first);
                res_y.push_back(map_atcoord[id].second);
                res_f.push_back(1);
            } else {
                res_x.push_back(0.0f);
                res_y.push_back(0.0f);
                res_f.push_back(0);
            }
        }
        result_.ids = ids;
        result_.x   = res_x;
        result_.y   = res_y;
        result_.found = res_f;
        as_.setSucceeded(result_);
    }

    private:

    // Feedback callback function
    void feedbackCallback(const FeedbackPtr& feedback) {
        ROS_INFO("Current robot status: %s", feedback->status.c_str());
    }

    // Result callback function
    void resultCallback(const actionlib::SimpleClientGoalState& state, const ResultPtr& result) {
        if (result->reached)
            ROS_INFO("Waypoint reached successfully!");
        else
            ROS_INFO("Waypoint NOT reached as planned.");
    }

    float distanceSquared(float x1, float y1, float x2, float y2) {
        return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
    }

    std::pair<float,float>  get_robot_pos() {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);

        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();

        std::pair<float, float> pos = {x, y};

        return pos;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_B");
    
    if (argc < 2) {
        ROS_ERROR("Error: At least one parameter is required.");
        ROS_INFO("Usage: %s <mcl_flag> ", argv[0]);
        return 1; 
    }

    std::string arg1(argv[1]);
    bool mcl_flag = (arg1 == "1" || arg1 == "true");

    Coordinator Coordinator("Apriltag_Search", mcl_flag);
    ros::spin();
    return 0;
}