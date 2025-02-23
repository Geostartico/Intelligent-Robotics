#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <cstdint>
#include "assignment1/map_waypoints.h"
#include "assignment1/apriltag_detect.h"
#include "assignment1/WaypointMoveAction.h"
#include "assignment1/ApriltagSearchAction.h"

// Maximum x coordinate of corridor space
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

    // Method that coordinates the search of the IDs given in the action goal
    void executeCB(const assignment1::ApriltagSearchGoalConstPtr &goal)
    {
        // Data structures to store the search status
        std::vector<int> ids = goal->ids;
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
        // Side length (meters) of a squared area used to divide environment space and obtain waypoints
        const float TILE_DIM = 2;
        srv.request.dim = TILE_DIM;

        // Waypoints are stored locally and filtered if the custom MCL manages the navigation through the corridor
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

        waypointsSorting(waypoints, TILE_DIM);
        ROS_INFO("WAYPOINTS TEST");
        for(auto w : waypoints) {
            ROS_INFO("x:%f y:%f", w.first, w.second);
        }

        feedback_.status = {"Waypoints loaded, Robot starts the navigation."};
        as_.publishFeedback(feedback_);

        // The current robot position is obtained
        std::pair<float,float> curr_pos = get_robot_pos();
        ROS_INFO("Initial position, x:%f y:%f", curr_pos.first, curr_pos.second);

        // Action client to send waypoints to the Movement Handler server
        WaypointMoveClient ac("move_to_goal", true);
        ROS_INFO("Waiting for Movemente_Handler server to start...");
        ac.waitForServer();
        ROS_INFO("Movemente_Handler server started. Now looking for the closest waypoint.");

        std::vector<std::pair<float, float>> waypoints_backup = waypoints;
        int counter = 0, searches_counter = 0;
        bool breaked_flag = false;
        const int MAX_SEARCHES = 2;

        while(waypoints.size() > 0) {

            // The next waypoint is selected
            auto closest_it = waypoints.begin();

            assignment1::WaypointMoveGoal wp_goal;
            wp_goal.x = (*closest_it).first;
            wp_goal.y = (*closest_it).second;
            ROS_INFO("Next Waypoint x:%f y:%f", wp_goal.x, wp_goal.y);

            // If the custom MCL is enabled, give a different feedback
            if(custom_mcl_flag) {
                feedback_.status = {"Before processing the first waypoint, the robot looks around the initial position and traverses the corridor using the custom Movement Control Law while scanning the environment."};
                as_.publishFeedback(feedback_);
                custom_mcl_flag = false;
            }
            else {
                feedback_.status = {"Robot moves towards the next waypoint while scanning the environment."};
                as_.publishFeedback(feedback_);
            }

            // The selected waypoint is sent as goal to the Movement Handler action server
            ac.sendGoal(
                wp_goal,
                boost::bind(&Coordinator::resultCallback, this, _1, _2),  
                WaypointMoveClient::SimpleActiveCallback(),
                boost::bind(&Coordinator::feedbackCallback, this, _1)
            );

            bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));
            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Waypoint processing result: %s", state.toString().c_str());
            } else {
                ROS_INFO("Waypoint not processed before the timeout.");
            }

            // The current robot position is updated
            // curr_pos = get_robot_pos();

            // The processed waypoint is removed
            waypoints.erase(closest_it);
            
            ++counter;
            feedback_.status = {"Robot has processed a waypoint (" + std::to_string(counter) + "/" + std::to_string(waypoints_backup.size()) + ")."};
            as_.publishFeedback(feedback_);

            feedback_.status = {"Robot looks around itself by doing a 360 degrees turn."};
            as_.publishFeedback(feedback_);

            // Client to receive the current status of the AprilTags Detection 
            ros::ServiceClient ad_client = nh_.serviceClient<assignment1::apriltag_detect>("apriltags_detected_service");
            assignment1::apriltag_detect ad_srv;

            // Update the local data structures with the informations returned via service
            if(ad_client.call(ad_srv)) {
                ROS_INFO("Call to apriltags_detected_service: SUCCESSFUL.");
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

            // Action feedback for how many and which tags have been found 
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

            // Stop the search if all tags have been found
            if(!(ids.size() - ids_counter)) {
                breaked_flag = true;
                break;
            }

            // Increase the search attemps counter is all waypoints were processed
            if(waypoints.empty())
                searches_counter++;

            // If all waypoints were processed and NOT every AprilTag has been found, restart the search doing the inverse path (max 3 attempts in total)
            if(searches_counter<MAX_SEARCHES && waypoints.empty() && (ids.size() - ids_counter)) {
                ROS_INFO("Search Attempt %d/%d: All waypoints processed but NOT all AprilTags found. Restarting the search following the inverse path.", searches_counter , MAX_SEARCHES);
                feedback_.status = {"Robot Search Attempt " + std::to_string(searches_counter + 1) + "/" + std::to_string(MAX_SEARCHES) +
                                    ": All waypoints processed but NOT all AprilTags found. Restarting the search following the inverse path.."};
                as_.publishFeedback(feedback_);
                std::reverse(waypoints_backup.begin(), waypoints_backup.end());
                waypoints = waypoints_backup;
                counter = 0;
            }
        }

        // Conclusive action feedback
        if(breaked_flag) {
            ROS_INFO("All AprilTags detected, coordinator node shuts down.");
            feedback_.status = {"Robot found all wanted AprilTags. Navigation and Scanning operations completed."};
            as_.publishFeedback(feedback_);
        }
        else {
            ROS_INFO("All Waypoints processed, coordinator node shuts down.");
            feedback_.status = {"Robot processed every waypoint. Navigation and Scanning operations completed."};
            as_.publishFeedback(feedback_);
        }
        ros::Duration(0.5).sleep();

        // Action result load with obtained data and sending
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

    // Method to obtain current robot position in the map reference frame
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

    // Method to sort the waypoints to obtain and efficient ordering (snake pattern)
    void waypointsSorting(std::vector<std::pair<float, float>>& waypoints, int TILE_DIM) {
        std::vector<std::vector<std::pair<float, float>>> tmp;
        for(auto w : waypoints) {
            int index = round(w.first / (TILE_DIM/2));
            while(index >= tmp.size())
                tmp.push_back(std::vector<std::pair<float, float>>());
            tmp[index].push_back(w);
        }
        waypoints.clear();
        int counter = 0;
        for(auto ws : tmp) {
            if(counter % 2 != 0)
                std::sort(ws.begin(), ws.end(), [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                return a.second < b.second; 
            });
            else
            std::sort(ws.begin(), ws.end(), [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                return a.second > b.second; 
            });
            if(!ws.empty()) {
                for(auto w : ws) waypoints.push_back(w);
                counter++;
            }
        }
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

    // Flag to enable the custom Motion Control Law
    std::string arg1(argv[1]);
    bool mcl_flag = (arg1 == "1" || arg1 == "true");

    Coordinator Coordinator("Apriltag_Search", mcl_flag);
    ros::spin();
    return 0;
}