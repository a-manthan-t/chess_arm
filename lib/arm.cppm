module;

#include <algorithm>
#include <array>
#include <format>
#include <functional>
#include <numbers>
#include <ranges>
#include <string>

#ifdef TESTING
#include "doctest.h"
#endif

// Handles serial kinematic chains made up of revolute joints.
export module arm;

import quaternion;

export namespace arm {
    using namespace quaternion;

    struct Orientation {
        Quaternion position, rotation;

        constexpr Orientation(Quaternion position, Quaternion rotation) : position { position }, rotation { rotation } {}
    };

    struct Joint {
        // The position vector of the tip of the arm segment from the local origin when angle = 0.
        const Quaternion armSegment;
        const Axis rotAxis;
        float angle {};

        Joint(Axis armAxis, float length, Axis rotAxis) : armSegment { vector(armAxis, length) }, rotAxis { rotAxis } {}
    };

    template <size_t size, size_t nwrist> // nwrist is the number of joints (from the right) that form the wrist.
    class Arm {
        static_assert(size >= nwrist, "There needs to be at least as many joints in the arm as wrist joints.");

        static constexpr Orientation ORIGIN { ZERO, IDENTITY };

        std::array<Joint, size> joints;
        Orientation base;

        public:
            static constexpr float DEFAULT_GRANULARITY { 0.001 };

            Arm(std::array<Joint, size> joints, Orientation base = ORIGIN) : joints { joints }, base { base } {}

            // Retrieve the angle of every joint for dispatch to the arm's microcontroller.
            std::array<float, size> collectAngles() {
                std::array<float, size> result {};

                int i {};
                for (const Joint& joint : joints) {
                    result[i++] = joint.angle;
                }

                return result;
            }

            /* Forward kinematics. */

            // Find the global orientation of the end effector.
            Orientation locateEndEffector() const {
                Orientation result { ORIGIN };

                for (const Joint& joint : joints | std::views::reverse) {
                    Quaternion jointRotation { rotation(joint.rotAxis, joint.angle) };
                    result = {
                        apply(jointRotation, joint.armSegment + result.position),
                        result.rotation = jointRotation * result.rotation
                    };
                }

                return {
                    apply(base.rotation, base.position + result.position),
                    result.rotation = base.rotation * result.rotation
                };
            }

            /* Inverse kinematics. */

            // The error in position should be allowed to dominate - the rotation error will be adjusted separately
            // if a wrist is provided.
            float errorTo(const Orientation& target) const {
                auto [position, rotation] { locateEndEffector() };
                return (position - target.position).magnitude() / position.magnitude()
                     + std::acosf(dot(rotation, target.rotation)) / std::numbers::pi_v<float>;
            }

            // Use CCD to set the arm's angles to those which bring the end effector as close as possible to the
            // target orientation (local minimum). The wrist is adjusted separately from the rest of the arm.
            void ccdTo(const Orientation& target, float granularity = DEFAULT_GRANULARITY) {
                auto adjust = [this, target, granularity](auto joints) mutable {
                    float lastError { std::numeric_limits<float>::infinity() };

                    for (int i { 0 }; i < 1'000'000; ++i) { // To avoid an infinite loop with while (true)
                        for (Joint& joint : joints) {
                            float error { errorTo(target) };
                            joint.angle += granularity;

                            if (errorTo(target) > error)
                                joint.angle -= 2 * granularity;
                        }

                        // Stop if function has converged (i.e., consecutive errors are withing 10^-6 of each other -
                        // this is used instead of the conventional proximity to the target.
                        if (float error { errorTo(target) }; lastError - error < std::pow(10, -6)) break;
                        else lastError = error;
                    }
                };

                adjust(joints | std::views::take(size - nwrist));
                adjust(joints | std::views::drop(size - nwrist));
            }
    };
}


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
