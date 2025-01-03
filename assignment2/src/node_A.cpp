#include <ros/ros.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include "assignment2/object_detect.h"
#include "assignment2/apriltag_detect.h"
#include "movement.h"

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

    // ros::ServiceClient detection_client = n.serviceClient<assignment2::object_detect>("/detection_srv");
    // assignment2::object_detect srv_d;
    // srv_d.request.ready = true;

    // // Request to start the detection routine and get the object position and preferred docking stations
    // std::vector<float> objs_x, objs_y;
    // std::vector<int> ids;
    // if(detection_client.call(srv_d)) {
    //     ROS_INFO("detection_srv call successful.");
    //     objs_x = srv_d.response.objs_x;
    //     objs_y = srv_d.response.objs_y;
    //     ids = srv_d.response.ids;
    // }
    // else {
    //     ROS_ERROR("Failed to call service detection_srv.");
    //     return 1;
    // }

    ros::Duration wait_time(2.0);

    ROS_INFO("Initializing Movement object. Robot takes position at dock 1.");
    Movement mov;
    wait_time.sleep();
    ros::ServiceClient ad_client = n.serviceClient<assignment2::apriltag_detect>("apriltags_detected_service");
    assignment2::apriltag_detect ad_srv;

    for(int i=2; i<=6; i++) {
        ROS_INFO("Moving to dock %u.", i);
        mov.goAround(i);
        wait_time.sleep();
    }
    ad_srv.request.create_collisions = true;
    ad_client.call(ad_srv);

    for(int i=5; i>=1; i--) {
        ROS_INFO("Moving to dock %u.", i);
        mov.goAround(i);
        wait_time.sleep();
    }

    return 0;
}
