module;

#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>

export module camera;

import arm;

export namespace camera {
    class Camera {
        cv::VideoCapture camera { 0, cv::CAP_ANY }; // replace 0 w/ PIPELINE from CMakeLists.txt
        cv::Mat raw, frame;

        public:
            std::mutex cameraMutex;
            std::vector<unsigned char> buffer;

            static Camera instance; // We only want a single shared camera.

            void process();
            [[noreturn]] void loop(const arm::Arm* robot);
    };
}
