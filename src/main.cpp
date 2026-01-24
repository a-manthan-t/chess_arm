#include <charconv>
#include <fstream>
#include <print>
#include <thread>

import quaternion;
import streamer;
import camera;
import arm;

using namespace quaternion;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else

int main(int argc, char* argv[]) {
    using namespace arm;
    using namespace camera;

    if (argc < 2) {
        std::println(stderr, "Please provide a configuration file.");
        return 1;
    }

    std::ifstream config { argv[1] };

    if (!config) {
        std::println(stderr, "Could not open configuration file.");
        return 1;
    }

    std::string line, token;

    /* Get stream configuration. */

    if (!std::getline(config, line)) {
        std::println(stderr, "Unexpected configuration file end.");
        return 1;
    }

    std::istringstream lineStream { line };
    std::vector<std::string> streamConfig;

    while (std::getline(lineStream, token, ';')) {
        streamConfig.push_back(token);
    }

    /* Get the number of wrist joints in the arm. */

    if (!std::getline(config, line)) {
        std::println(stderr, "Unexpected configuration file end.");
        return 1;
    }

    size_t wristSize;
    auto [_, err] = std::from_chars(line.data(), line.data() + line.size(), wristSize);

    if (err != std::errc {}) {
        std::println(stderr, "Could not parse wrist size.");
        return 1;
    }

    /* Get the descriptions of each joint in the arm. */

    std::vector<Joint> joints;

    while (std::getline(config, line)) {
        if (line.size() < 5) { // Should at least be A;_;B.
            std::println(stderr, "Could not parse joints.");
            return 1;
        }

        float segmentLength;
        auto [_, err] = std::from_chars(line.data() + 1, line.data() + line.size() - 1, segmentLength);

        if (err != std::errc {}) {
            std::println(stderr, "Could not parse joints.");
            return 1;
        }

        Axis armAxis { line.front() == 'X' ? Axis::X : line.front() == 'Y' ? Axis::Y : Axis::Z };
        Axis rotAxis { line.back() == 'X' ? Axis::X : line.back() == 'Y' ? Axis::Y : Axis::Z };

        joints.emplace_back(armAxis, segmentLength, rotAxis);
    }

    if (joints.size() < wristSize) {
        std::println(stderr, "Wrist size is greater than number of joints.");
        return 1;
    }

    /* Configure and start the robot, vision, and streaming threads. */

    Arm robot { joints, wristSize };
    Camera cam { &robot };

    std::thread arm { &Arm::follow, &robot };
    std::thread vision { &Camera::loop, &cam };

    if (unsigned int fps; streamConfig.size() == 3) {
        auto [_, err] = std::from_chars(streamConfig[2].data(), streamConfig[2].data() + streamConfig[2].size(), fps);

        if (err == std::errc {}) {
            std::thread stream { &streamer::stream, streamConfig[0], streamConfig[1], fps, &cam };
            stream.join();
        } else {
            std::println(stderr, "Could not parse FPS.");
        }
    }

    arm.join();
    vision.join();

    return 0;
}
#endif
