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

    std::string line;
    std::getline(config, line);
    // websocket_url;password;fps (empty if none)
    std::getline(config, line);
    // Line containing wrist size in int
    while (std::getline(config, line)) {
        // remaining are joints in X/Y/Z;length;rotAxis format
        std::println("{}", line);
    }

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
    Camera cam { &robot };

    // read these from files
    std::thread arm { &Arm::follow, &robot };
    std::thread vision { &Camera::loop, &cam };
    std::thread stream { &streamer::stream, "ws://localhost:8008", "very-secure-password", 15, &cam };

    arm.join();
    vision.join();
    stream.join();

    return 0;
}
#endif
