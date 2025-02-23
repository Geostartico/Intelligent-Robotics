#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "assignment1/ApriltagSearchAction.h"

// Typedefs for better readability
typedef actionlib::SimpleActionClient<assignment1::ApriltagSearchAction> ApriltagSearchClient;
typedef assignment1::ApriltagSearchFeedbackConstPtr FeedbackPtr;
typedef assignment1::ApriltagSearchResultConstPtr ResultPtr;

// Feedback callback function
void feedbackCallback(const FeedbackPtr& feedback) {
    for(auto s : feedback->status)
        ROS_INFO("%s", s.c_str());
}

// Result callback function
void resultCallback(const actionlib::SimpleClientGoalState& state, const ResultPtr& result) {
    ROS_INFO("Apriltags search results:");
    for(int i=0; i<result->ids.size(); i++) {
        if(result->found[i])
            ROS_INFO("AprilTag %u found at x:%f y:%f", result->ids[i], result->x[i], result->y[i]);
        else
            ROS_INFO("Apriltag %u NOT found", result->ids[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_A");
    ros::NodeHandle n;

    // Service client to obtain the IDs to find
    ros::ServiceClient ids_client = n.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;

    // Request the IDs
    std::vector<int> ids;
    if(ids_client.call(srv)) {
        ROS_INFO("ids_generator_node call successful.");
        ids = srv.response.ids;
        ROS_INFO("Received %lu AprilTag ids:", ids.size());
    }
    else {
        ROS_ERROR("Failed to call service ids_generator_node.");
        return 1;
    }

    for(int i : ids)
        ROS_INFO("Tag: %u", i);

    // Action client to request the search of the wanted IDs
    ApriltagSearchClient ac("Apriltag_Search", true);
    ROS_INFO("Waiting for Apriltag_Search server to start...");
    ac.waitForServer();
    ROS_INFO("Apriltag_Search server started, sending goal.");

    assignment1::ApriltagSearchGoal goal;
    goal.ids = ids;

    ac.sendGoal(goal, &resultCallback, ApriltagSearchClient::SimpleActiveCallback(), &feedbackCallback);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("AprilTags search completed: TASK %s", state.toString().c_str());
    } else {
        ROS_INFO("AprilTags search NOT finished before timeout.");
    }
        
    return 0;
}