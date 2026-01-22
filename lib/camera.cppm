module;

#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>

export module camera;

export namespace camera {
    class Camera {
        cv::VideoCapture camera { 0, cv::CAP_ANY }; // replace 0 w/ PIPELINE
        cv::Mat raw, frame;

        public:
            std::mutex cameraMutex;
            std::vector<unsigned char> buffer;

            static Camera instance; // We only want one camera.

            void capture();
            void encode();
    };
}
