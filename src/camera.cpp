module;

#include <mutex>

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

import arm;

namespace camera {
    Camera Camera::instance {};

    void Camera::process() {
        if (camera.read(raw)) {
            frame = raw; // Change for processing
        }
    }

    [[noreturn]] void Camera::loop(const arm::Arm* robot) {
        while (true) {
            process();

            // Update robot

            // For streaming purposes.
            if (!frame.empty()) {
                std::lock_guard lock { cameraMutex }; // Lock so streamer doesn't read a partially ready image.
                cv::imencode(".jpg", frame, buffer);
            }
        }
    }
}
