module;

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

namespace camera {
    void Camera::initialise(unsigned int maxFps) {
        float time {};
    }
    cv::VideoCapture Camera::camera { PIPELINE, cv::CAP_ANY };
}
