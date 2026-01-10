module;

#include <algorithm>
#include <array>
#include <format>
#include <numbers>

#ifdef TESTING
#include "../lib/doctest.h"
#endif

// Handles forward and inverse kinematics for serial kinematic chains made up of revolute joints.
module arm;

import quaternion;


#ifdef TESTING
TEST_SUITE("Arm Tests") {
    // Tests involving CCD are of course dependent on the target being reachable with the degrees of freedom given.
    // Watch out for gimbal lock cases during testing (different Euler angle representations possible).

    using namespace quaternion;
    using namespace arm;
    using std::numbers::pi;

    std::array joints {
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

    Orientation target1 { vector(5, 1.3, 6), rotation(pi/3, pi/6, pi/4) };
    Orientation target2 { vector(2, 3, 4), rotation(0, pi/9, pi/2.5) };
    Orientation target3 { vector(3, -2, 1), rotation(pi/8, pi/5, pi/7) };

    Arm<9, 3> arm { joints };

    TEST_CASE("Collect angles") {
        CHECK(arm.collectAngles() == std::array<float, 9> {});
    }

    TEST_CASE("Forward kinematics") {
        // If inverse kinematics tests succeed, this function is guaranteed to be working too.
        Orientation
            orientation { arm.locateEndEffector() },
            expected { vector(3, 1, 7), IDENTITY };

        CHECK(orientation.position == expected.position);
        CHECK(orientation.rotation == expected.rotation);
    }

    TEST_CASE("Inverse kinematics") {
        Arm _arm = arm; // Local copy since it gets updated.

        _arm.ccdTo(target1);
        CHECK(_arm.errorTo(target1) < 0.0025);

        _arm.ccdTo(target2);
        CHECK(_arm.errorTo(target2) < 0.0025);

        _arm.ccdTo(target3);
        CHECK(_arm.errorTo(target3) < 0.0025);
    }
}
#endif
