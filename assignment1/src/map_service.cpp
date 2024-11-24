#include "ros/ros.h"
#include "assignment1/map_service.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

geometry_msgs::Pose get_global_pose(int px, int py, nav_msgs::OccupancyGrid::ConstPtr& map){
    int w = map->info.width;
    int h = map->info.height;
    geometry_msgs::Pose origin = map->info.origin;
    double resolution = map->info.resolution;
    double wx = origin.position.x + px * resolution;
    double wy = origin.position.y + py * resolution;
    geometry_msgs::Pose pose;
    pose.position.x = wx;
    pose.position.y = wy;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
}

bool map(assignment1::map_service::Request &req,
        assignment1::map_service::Response &res)
{
    int id = req.id;
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", ros::Duration(10.0));
    if(msg == nullptr){
        ROS_ERROR("unable to retrieve costmap");
        return false;
    }
    else{
        //TODO
    }
    ROS_INFO("request: id=%d", id);
    ROS_INFO("sending back response: []");
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_service");
    ros::NodeHandle n;
    //creates a service that makes the service available
    ros::ServiceServer service =
        n.advertiseService("map_service", map);
    ROS_INFO("map available to be read");
    ros::spin();
    return 0;
}
