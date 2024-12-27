#include <ros/ros.h>
#include <tiago_iaslab_simulation/Coeffs.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_A");
    ros::NodeHandle n;

    // Service client to obtain the parameters of the line on which the objects shall be placed
    ros::ServiceClient coeffs_client = n.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    tiago_iaslab_simulation::Coeffs srv;
    srv.request.ready = true;

    // Request the coeffs
    std::vector<float> coeffs;
    if(coeffs_client.call(srv)) {
        ROS_INFO("straight_line_srv call successful.");
        coeffs = srv.response.coeffs;
    }
    else {
        ROS_ERROR("Failed to call service straight_line_srv.");
        return 1;
    }

    ROS_INFO("Line parameters, m:%f q:%f", coeffs[0], coeffs[1]);

    // Placing Routine

    return 0;
}
