module;

#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>

export module camera;

import arm;

export namespace camera {
    class Camera {
        cv::VideoCapture camera { PIPELINE, cv::CAP_ANY };
        cv::Mat raw, frame;

        void processRaw();
        void generateCheckpoints();
        void encodeFrame();

        public:
            Camera(arm::Arm* robot) : robot(robot) {}

            std::mutex cameraMutex;
            std::vector<unsigned char> buffer;
            arm::Arm* robot;

            [[noreturn]] void loop();
    };
}
