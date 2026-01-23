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

    if (argc < 2) {
        std::println(stderr, "Please provide a configuration file.");
        return 1;
    }

    std::ifstream config { argv[1] };

    if (!config) {
        std::println(stderr, "Could not open configuration file.");
        return 1;
    }

    std::string line;
    while (std::getline(config, line)) {
        std::println("{}", line);
    }

     // Actually use the file output.

    std::vector joints {
        Joint { Axis::Z, 1, Axis::Z },
        Joint { Axis::Z, 3, Axis::Y },
        Joint { Axis::Y, 1, Axis::X },
        Joint { Axis::X, 2, Axis::Z },
        Joint { Axis::Z, 3, Axis::Y },
        Joint { Axis::X, 1, Axis::X },
        Joint { Axis::X, 0, Axis::Z },
        Joint { Axis::X, 0, Axis::Y },
        Joint { Axis::X, 0, Axis::X }
    };

    Arm robot { joints, 3 };

    std::thread stream { &streamer::stream, "ws://localhost:8008", "very-secure-password", 15, &robot }; // read these from files
    std::thread camera { &camera::Camera::loop, &camera::Camera::instance, &robot };
    std::thread arm { &Arm::follow, &robot };

    stream.join();
    camera.join();
    arm.join();

    return 0;
}
#endif
