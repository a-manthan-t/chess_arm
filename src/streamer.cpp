module;

#include <chrono>
#include <mutex>
#include <print>
#include <thread>

#include "../lib/easywsclient.hpp"

// Streams the camera data to a WebSocket server and receives emergency stop commands.
module streamer;

import quaternion;
import arm;
import camera;

namespace streamer {
    using easywsclient::WebSocket;

    // Run the stream.
    [[noreturn]] void stream(const std::string& url, const std::string& token, unsigned int fps, camera::Camera* cam) {
        std::unique_ptr<WebSocket> ws;
        std::string streamToken { "promote;" + token };
        int tpf { static_cast<int>(1000.f/static_cast<float>(fps)) }; // Fps to milliseconds per frame.

        // Keep trying to connect and stream frames.
        while (true) {
            ws.reset(WebSocket::from_url(url));

            if (ws) { // If connected...
                ws->send(streamToken);
                ws->poll();             // Send token to gain stream access.
                ws->poll(-1);    // Receive response.

                ws->dispatch([url](const std::string& message) {
                    if (message == "promotion_success") {
                        std::println("Connected to server: {}", url);
                    } else if (message == "promotion_fail") {
                        std::println("Incorrect token provided.", url);
                    }
                });

                while (ws->getReadyState() != WebSocket::CLOSED) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(tpf));

                    // Intercept emergency stop commands.
                    ws->poll();
                    ws->dispatchBinary([cam](const std::vector<unsigned char>& message) {
                        std::array<float, 4> floats {};
                        std::memcpy(floats.data(), message.data(), message.size()); // Convert bytes to floats.

                        cam->robot->stop(quaternion::vector(floats[0], floats[1], floats[2]), floats[3] == 1);
                    });

                    std::lock_guard lock { cam->cameraMutex }; // Lock so camera doesn't change buffer.
                    ws->sendBinary(cam->buffer);
                }
            }

            std::println("Lost stream access to the server. Attempting to reconnect in 5s...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}
