#include <opencv2/opencv.hpp>
#include <iostream>
#include <queue>

using namespace cv;
using namespace std;

void processHalf(const Mat& binaryImage, Rect region, int threshold, queue<Rect>& componentQueue) {
    // Extract the half region of interest (ROI)
    Mat roi = binaryImage(region);

    // Apply connected components within the ROI
    Mat labels, stats, centroids;
    int numComponents = connectedComponentsWithStats(roi, labels, stats, centroids);

    // Process each subcomponent in the ROI
    for (int i = 1; i < numComponents; ++i) { // Start from 1 (skip background)
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);

        // Map the coordinates back to the original image space
        Rect subRect(region.x + x, region.y + y, width, height);

        // Check if the subcomponent is non-empty
        Mat subRoi = roi(Rect(x, y, width, height));
        if (countNonZero(subRoi) > 0) {
            componentQueue.push(subRect);
        }
    }
}

void processConnectedComponents(const Mat& binaryImage, int threshold) {
    Mat labels, stats, centroids;
    int numComponents = connectedComponentsWithStats(binaryImage, labels, stats, centroids);

    queue<Rect> componentQueue;

    // Enqueue all initial components
    for (int i = 1; i < numComponents; ++i) { // Start from 1 (skip background)
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        componentQueue.push(Rect(x, y, width, height));
    }

    // Iteratively process each component
    while (!componentQueue.empty()) {
        Rect component = componentQueue.front();
        componentQueue.pop();

        int width = component.width;
        int height = component.height;

        Mat roi = binaryImage(component);

        // Skip empty rectangles
        if (countNonZero(roi) == 0) {
            continue;
        }

        cout << "Processing component at (" << component.x << ", " << component.y
             << ") with size " << width << "x" << height << endl;

        // If dimensions exceed threshold, divide and process halves
        if (width > threshold || height > threshold) {
            if (width >= height) {
                // Split horizontally into left and right halves
                int midX = component.x + width / 2;
                Rect leftHalf(component.x, component.y, width / 2, height);
                Rect rightHalf(midX, component.y, width - width / 2, height);

                processHalf(binaryImage, leftHalf, threshold, componentQueue);
                processHalf(binaryImage, rightHalf, threshold, componentQueue);
            } else {
                // Split vertically into top and bottom halves
                int midY = component.y + height / 2;
                Rect topHalf(component.x, component.y, width, height / 2);
                Rect bottomHalf(component.x, midY, width, height - height / 2);

                processHalf(binaryImage, topHalf, threshold, componentQueue);
                processHalf(binaryImage, bottomHalf, threshold, componentQueue);
            }
        } else {
            // Component size is within threshold; process it
            Mat processedComponent = binaryImage(component);
            Mat result;
            cvtColor(binaryImage, result, COLOR_GRAY2BGR);
            rectangle(result, component, Scalar(0, 255, 0), 2);
            imshow("Component", result);
            waitKey(0);
        }
    }
}

int main() {
    // Load binary image
    Mat binaryImage = imread("/tmp/map.png", IMREAD_GRAYSCALE);
    if (binaryImage.empty()) {
        cerr << "Could not load image!" << endl;
        return -1;
    }

    // Threshold the image to ensure binary (if not already binary)
    threshold(255 - binaryImage, binaryImage, 128, 255, THRESH_BINARY);

    int dimensionThreshold = 50; // Example threshold
    processConnectedComponents(binaryImage, dimensionThreshold);

    return 0;
}
