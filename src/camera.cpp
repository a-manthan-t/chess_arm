module;

#include <mutex>

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

namespace camera {
    Camera Camera::instance {};

    void Camera::capture() {
        camera >> raw;
        // process
        camera >> frame; // Use processed image instead
    }

    void Camera::encode() {
        if (!frame.empty()) {
            std::lock_guard lock { cameraMutex };
            cv::imencode(".jpg", frame, buffer);
        }
    }
}
