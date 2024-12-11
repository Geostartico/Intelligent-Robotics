#include "ros/ros.h"
#include "assignment1/map_waypoints.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <queue>

using namespace cv;

//transform map pixel coordinates into map world coordinates
geometry_msgs::Pose get_global_pose(int pixel_x, int pixel_y, nav_msgs::OccupancyGrid::ConstPtr& map){
    int width = map->info.width;
    int height = map->info.height;
    geometry_msgs::Pose origin = map->info.origin;
    double resolution = map->info.resolution;
    double world_x = origin.position.x + pixel_x * resolution;
    double world_y = origin.position.y + pixel_y * resolution;
    geometry_msgs::Pose pose;
    pose.position.x = world_x;
    pose.position.y = world_y;
    pose.position.z = 0;
    //to have a proper orientation for move_base
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
}

//process one half of the connected component
void processHalf(const Mat& binaryImage, Rect region, int threshold, std::queue<Rect>& componentQueue) {
    //ROS_INFO("processing x:%d, y:%d, width:%d, height:%d",region.x ,region.y, region.width, region.height);
    Mat roi = binaryImage(region);

    Mat labels, stats, centroids;
    int numComponents = connectedComponentsWithStats(roi, labels, stats, centroids);
    
    //compute the connected components within it
    for (int i = 1; i < numComponents; ++i) {
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);

        Rect subRect(region.x + x, region.y + y, width, height);

        //add the roi only if it is not empty
        Mat subRoi = roi(Rect(x, y, width, height));
        if (countNonZero(subRoi) > 0) {
            componentQueue.push(subRect);
        }
    }
}

void processConnectedComponents(const Mat& binaryImage, int threshold, std::vector<float>& x, std::vector<float>& y, nav_msgs::OccupancyGrid::ConstPtr& map) {
    //compute connected components
    Mat labels, stats, centroids;
    int numComponents = connectedComponentsWithStats(binaryImage, labels, stats, centroids);

    std::queue<Rect> componentQueue;
    ROS_INFO("processing components");
    //0 is background
    for (int i = 1; i < numComponents; ++i) { 
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        componentQueue.push(Rect(x, y, width, height));
    }
    ROS_INFO("components processed");

    while (!componentQueue.empty()) {
        Rect component = componentQueue.front();
        componentQueue.pop();

        int width = component.width;
        int height = component.height;

        Mat roi = binaryImage(component);

        //skip component if empty
        if (countNonZero(roi) == 0) {
            continue;
        }

        //if it is greater than the threshold recursively divide in half
        if (width > threshold || height > threshold) {
            if (width >= height) {
                int midX = component.x + width / 2;
                Rect half1(component.x, component.y, width / 2, height);
                Rect half2(midX, component.y, width - width / 2, height);

                processHalf(binaryImage, half1, threshold, componentQueue);
                processHalf(binaryImage, half2, threshold, componentQueue);
            } else {
                int midY = component.y + height / 2;
                Rect half1(component.x, component.y, width, height / 2);
                Rect half2(component.x, midY, width, height - height / 2);

                processHalf(binaryImage, half1, threshold, componentQueue);
                processHalf(binaryImage, half2, threshold, componentQueue);
            }
        } else {
            //return the component
            geometry_msgs::Pose pose = get_global_pose(component.x+width/2, component.y+height/2, map);
            x.push_back(pose.position.x);
            y.push_back(pose.position.y);
        }
    }
}
bool map_waypoints(assignment1::map_waypoints::Request &req,
        assignment1::map_waypoints::Response &res)
{
    //obtain map
    float dim = req.dim;
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(10.0));
    if(msg == nullptr){
        ROS_ERROR("unable to retrieve map");
        return false;
    }
    else{
        //tranform map into opencv Mat
        int width = msg->info.width;
        int height = msg->info.height; 
        cv::Mat image(height, width, CV_8UC1);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int index = i * width + j;
                image.at<uchar>(i, j) = msg->data[index];  
            }
        }
        //invert image to process connected components correctly
        image = 255 -image;

        processConnectedComponents(image, dim/msg->info.resolution, res.x, res.y, msg);
    }
    ROS_INFO("sending back response");
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_waypoints_service");
    ros::NodeHandle n;
    //creates a service that makes the service available
    ros::ServiceServer service =
        n.advertiseService("map_waypoints_service", map_waypoints);
    ROS_INFO("map available to be read");
    ros::spin();
    return 0;
}
