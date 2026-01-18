#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>

import quaternion;
import arm;

using namespace quaternion;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main() {
    // PIPELINE defined in CMakeLists.txt.
    cv::VideoCapture cap { PIPELINE, cv::CAP_ANY };

    std::this_thread::sleep_for(std::chrono::seconds(2));

    cv::Mat img;
    cap >> img;

    cv::Mat thresh;
    cv::threshold(img, thresh, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(
        thresh,
        contours,
        hierarchy,
        cv::RETR_TREE,
        cv::CHAIN_APPROX_SIMPLE
    );

    std::cout << "Found " << contours.size() << " contours." << std::endl;

    std::vector<std::vector<cv::Point>> filteredContours;
    for (const std::vector<cv::Point>& c : contours) {
        double area = cv::contourArea(c);
        if (area > 100) { // only keep contours larger than 100 pixels
            filteredContours.push_back(c);
        }
    }

    std::cout << "Filtered to " << filteredContours.size() << " contours." << std::endl;

    cv::Mat output = img.clone();
    cv::drawContours(output, filteredContours, -1, cv::Scalar(0, 255, 0), 2);

    cv::imwrite("image.jpg", output);

    return 0;
}
#endif
