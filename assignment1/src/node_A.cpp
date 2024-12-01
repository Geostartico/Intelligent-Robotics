#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_A");
    ros::NodeHandle n;

    ros::ServiceClient ids_client = n.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;

    std::vector<int> ids;
    if(ids_client.call(srv)) {
        ROS_INFO("ids_generator_node call successful.");
        ids = srv.response.ids;
        ROS_INFO("Received %lu AprilTag ids.", ids.size());
    }
    else {
        ROS_ERROR("Failed to call service ids_generator_node.");
        return 1;
    }

    for(int i : ids)
        ROS_INFO("Tag: %lu", i);
        
    return 0;
}