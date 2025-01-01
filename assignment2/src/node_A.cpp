#include <ros/ros.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include "assignment2/object_detect.h"

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

    // Placing Routine

    ros::ServiceClient detection_client = n.serviceClient<assignment2::object_detect>("/detection_srv");
    assignment2::object_detect srv_d;
    srv_d.request.m = coeffs[0];
    srv_d.request.q = coeffs[1];
    srv_d.request.ready = true;

    // Request to start the detection routine and get the object position and preferred docking stations
    std::vector<float> objs_x, objs_y, docks_x, docks_y;
    std::vector<int> ids;
    if(detection_client.call(srv_d)) {
        ROS_INFO("detection_srv call successful.");
        objs_x = srv_d.response.objs_x;
        objs_y = srv_d.response.objs_y;
        docks_x = srv_d.response.docks_x;
        docks_y = srv_d.response.docks_y;
        ids = srv_d.response.ids;
    }
    else {
        ROS_ERROR("Failed to call service detection_srv.");
        return 1;
    }

    return 0;
}
