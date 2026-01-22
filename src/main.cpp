#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../lib/easywsclient.hpp"

import quaternion;
import arm;
import streamer;

using namespace quaternion;

import camera;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main() {
    // PIPELINE defined in CMakeLists.txt.
    cv::VideoCapture cap { 0, cv::CAP_ANY };

    std::thread t { streamer::stream, "ws://localhost:8008", "very-secure-password", 15 };

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        cv::Mat img;
        cap >> img;

        cv::Mat grey;
        cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);

        cv::Mat thresh;
        cv::threshold(grey, thresh, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(
            thresh,
            contours,
            hierarchy,
            cv::RETR_TREE,
            cv::CHAIN_APPROX_SIMPLE
        );

        std::vector<std::vector<cv::Point>> filteredContours;
        for (const std::vector<cv::Point>& c : contours) {
            double area = cv::contourArea(c);
            if (area > 100) { // only keep contours larger than 100 pixels
                filteredContours.push_back(c);
            }
        }

        cv::Mat output = img.clone();
        cv::drawContours(output, filteredContours, -1, cv::Scalar(0, 255, 0), 2);

        camera::Camera::instance.capture();
        camera::Camera::instance.encode();
    }

    t.join();

    return 0;
}
#endif
