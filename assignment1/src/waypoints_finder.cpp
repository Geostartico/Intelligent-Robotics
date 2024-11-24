#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;

void non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, bool remove_plateaus) {
    // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
    cv::dilate(image, mask, cv::Mat());
    cv::compare(image, mask, mask, cv::CMP_GE);

    // optionally filter out pixels that are equal to the local minimum ('plateaus')
    if (remove_plateaus) {
        cv::Mat non_plateau_mask;
        cv::erode(image, non_plateau_mask, cv::Mat());
        cv::compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
        cv::bitwise_and(mask, non_plateau_mask, mask);
    }
}

int main(int argc, char **argv){
   Mat src = imread("/home/cwinge/map.png", IMREAD_GRAYSCALE); 
   Mat dist;
   distanceTransform(src==0, dist, DIST_L2, 3);
   // Normalize the distance image for range = {0.0, 1.0}
   // so we can visualize and threshold it
   normalize(dist, dist, 0, 1.0, NORM_MINMAX);
   Mat max;
   non_maxima_suppression(dist, max, true);
   //imshow("Distance Transform Image", dist);
   //imshow("max", max);
   //imshow("map", src == 0);
   //imshow("given costmap", src );
   Mat points,kernel;
   kernel = (Mat_<char>(5, 5) << 
           1, 1, 1, 1, 1, 
           1, 1, 1, 1, 1, 
           1, 1, 1, 1, 1, 
           1, 1, 1, 1, 1, 
           1, 1, 1, 1, 1);
   morphologyEx(max, points, MORPH_CLOSE, kernel);
   imshow("waypoints", points );
   imshow("waypoints", points & (255 - src));
   waitKey(0);
}
