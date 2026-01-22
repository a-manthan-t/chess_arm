module;

#include <chrono>
#include <mutex>
#include <print>
#include <thread>

#include "../lib/easywsclient.hpp"

module streamer;

import camera;

namespace streamer {
    using easywsclient::WebSocket;

    [[noreturn]] void stream(const std::string& url, const std::string& token, unsigned int fps) {
        std::unique_ptr<WebSocket> ws;
        std::string streamToken { "promote;" + token };
        int tpf { static_cast<int>(1000.f/static_cast<float>(fps)) };

        while (true) {
            ws.reset(WebSocket::from_url(url));
            ws->send(streamToken);
            ws->poll();             // Send token.
            ws->poll(-1);    // Receive response.

            ws->dispatch([&](const std::string& message) {
                if (message == "success") {
                    std::println("Connected to server: {}", url);
                } else if (message == "fail") {
                    std::println("Incorrect token provided.", url);
                }
            });

            while (ws->getReadyState() != WebSocket::CLOSED) {
                ws->poll();
                std::this_thread::sleep_for(std::chrono::milliseconds(tpf)); // sep thread?
                ws->dispatch([&](const std::string& message) {
                    std::println("msg {}", message);
                    if (message.starts_with("command")) {

                    }
                });

                std::lock_guard lock { camera::Camera::instance.cameraMutex };
                ws->sendBinary(camera::Camera::instance.buffer);
            }

            std::println("Lost stream access to the server. Attempting to reconnect in 5s...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}
