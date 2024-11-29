#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vector>
#include "assignment1/map_waypoints.h"
#include "assignment1/WaypointMoveAction.h"

// Typedefs for better readability
typedef actionlib::SimpleActionClient<assignment1::WaypointMoveAction> WaypointMoveClient;
typedef assignment1::WaypointMoveFeedbackConstPtr FeedbackPtr;
typedef assignment1::WaypointMoveResultConstPtr ResultPtr;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinator_node");
    ros::NodeHandle n;

    // Service client to get the waypoints
    ros::ServiceClient wp_client = n.serviceClient<assignment1::map_waypoints>("map_waypoints_service");
    assignment1::map_waypoints srv;
    const float TILE_DIM = 2;
    srv.request.dim = TILE_DIM;

    std::vector<float> x, y;
    std::vector<std::pair<float, float>> waypoints;
    if(wp_client.call(srv)) {
        ROS_INFO("map_waypoints_service call successful.");
        x = srv.response.x;
        y = srv.response.y;
        for (int i=0; i<x.size(); i++) {
            waypoints.emplace_back(x[i], y[i]);
        }        
        ROS_INFO("Received %lu waypoints.", x.size());
        // ROS_INFO("Last Waypoint received, x:%f y:%f", x[x.size() - 1], y[y.size() - 1]);
    }
    else {
        ROS_ERROR("Failed to call service map_waypoints_service.");
        return 1;
    }

    std::pair<float,float> curr_pos = {0.0f, 0.0f};
    ROS_INFO("Initial position, x:%f y:%f", curr_pos.first, curr_pos.second);

    // Action client to send the waypoint goal
    WaypointMoveClient ac("move_to_goal", true);
    ROS_INFO("Waiting for robot server to start...");
    ac.waitForServer();
    ROS_INFO("Robot server started, sending first waypoint.");

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

        assignment1::WaypointMoveGoal goal;
        goal.x = (*closest_it).first;
        goal.y = (*closest_it).second;
        ROS_INFO("Next Waypoint x:%f y:%f", goal.x, goal.y);

        // Send the goal with result and feedback callbacks, and no active callback
        ac.sendGoal(goal, &resultCallback, WaypointMoveClient::SimpleActiveCallback(), &feedbackCallback);

        bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Waypoint reached: %s", state.toString().c_str());
        } else {
            ROS_INFO("Waypoint not reached before the timeout.");
        }

        // Update curr_pos with the closest point
        curr_pos = *closest_it;

        // Remove the closest point from the vector
        waypoints.erase(closest_it);
    }

    ROS_INFO("All Waypoints visited, coordinator node shuts down.");

    return 0;
}