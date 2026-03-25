module;

#include <mutex>
#include <vector>

#include <opencv2/videoio.hpp>

export module camera;

import arm;
import board;

export namespace camera {
    const cv::Scalar BLACK { 0, 0, 0 }, WHITE { 255, 255, 255 }, GREEN { 255, 255, 255 };

    class Camera {
        cv::VideoCapture camera { PIPELINE, cv::CAP_ANY };
        std::array<cv::Rect2f, 64> squares {};
        board::Board currentBoard {};
        cv::Mat raw, frame;
        bool whitesPerspective;

        board::Board processRaw();
        void generateCheckpoints();
        void encodeFrame();

        public:
            Camera(arm::Arm* robot, bool whitesPerspective) : robot(robot), whitesPerspective(whitesPerspective) {}

            std::mutex cameraMutex;
            std::vector<unsigned char> buffer;
            arm::Arm* robot;

            bool configure();
            [[noreturn]] void loop();
    };
}
