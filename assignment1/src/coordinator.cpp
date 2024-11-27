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

int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinator_node");
    ros::NodeHandle n;

    // Service client to get the waypoints
    ros::ServiceClient wp_client = n.serviceClient<assignment1::map_waypoints>("map_waypoints_service");
    assignment1::map_waypoints srv;
    const float TILE_DIM = 50;
    srv.request.dim = TILE_DIM;

    std::vector<float> x, y;
    if(wp_client.call(srv)) {
        ROS_INFO("map_waypoints_service call successful.");
        x = srv.response.x;
        y = srv.response.y;
        ROS_INFO("Received %lu waypoints.", x.size());
    }
    else {
        ROS_ERROR("Failed to call service map_waypoints_service.");
        return 1;
    }

    // Action client to send the waypoint goal
    WaypointMoveClient ac("WaypointMove", true);
    ROS_INFO("Waiting for robot server to start...");
    ac.waitForServer();
    ROS_INFO("Robot server started, sending first waypoint.");

    assignment1::WaypointMoveGoal goal;
    goal.x = x[x.size() - 1];
    goal.y = y[y.size() - 1];
    ROS_INFO("Heading towards x:%f y:%f", goal.x, goal.y);

    // Send the goal with result and feedback callbacks, and no active callback
    ac.sendGoal(goal, &resultCallback, WaypointMoveClient::SimpleActiveCallback(), &feedbackCallback);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Waypoint reached: %s", state.toString().c_str());
    } else {
        ROS_INFO("Waypoint not reached before the timeout.");
    }

    return 0;
}
