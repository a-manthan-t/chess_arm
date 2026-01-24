module;

#include <mutex>

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

import arm;

namespace camera {
    void Camera::processRaw() {
        if (camera.read(raw)) {
            // Detect board
            // Get coordinates of squares (relative to???) - maybe corners and interpolate is fine
            // Identify pieces (and target ones in different colours)
            // Highlight everything and save to frame
            frame = raw;
        }
    }

    void Camera::generateCheckpoints() {

    }

    // For streaming purposes.
    void Camera::encodeFrame() {
        if (!frame.empty()) {
            std::lock_guard lock { cameraMutex }; // Lock so streamer doesn't read a partially ready image.
            cv::imencode(".jpg", frame, buffer);
        }
    }

    [[noreturn]] void Camera::loop() {
        while (true) {
            processRaw();
            generateCheckpoints();
            encodeFrame();
        }
    }
}
