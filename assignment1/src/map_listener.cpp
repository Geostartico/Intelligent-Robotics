#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
void messageCallback(const
		nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int width = msg->info.width;
    int height = msg->info.height; ROS_INFO("got it");
    ROS_INFO("%d", width);
    ROS_INFO("%d", height);

    cv::Mat image(height, width, CV_8UC1);

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = i * width + j;
            int value = msg->data[index];

            image.at<uchar>(i, j) = value;  
            //if (value > 100) {
            //    image.at<uchar>(i, j) = 255;  
            //} else if (value > 0 && value < 100) {
            //    image.at<uchar>(i, j) = 0;    
            //} else if (value == -1) {
            //    image.at<uchar>(i, j) = 127;  
            //}
        }
    }

    cv::imwrite("/home/cwinge/map.png", image);
    ROS_INFO("got out");
}
int main(int argc, char **argv)
{
	//pass argc, argv and the name of the node
	ros::init(argc, argv, "listener");
    ROS_INFO("starting");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/map",
			1000, messageCallback);
	ros::spin();
	return 0;
}
