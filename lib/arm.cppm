module;

#include <array>
#include <format>
#include <numbers>
#include <ranges>

export module arm;

import quaternion;

export namespace arm {
    using namespace quaternion;

    struct Orientation {
        Quaternion position, rotation;

        constexpr Orientation(Quaternion position, Quaternion rotation) : position { position }, rotation { rotation } {}
    };

    struct Joint {
        const Quaternion armSegment; // The position vector to the arm segment tip from the local origin when angle == 0.
        const Axis rotAxis;
        float angle {};

        Joint(Axis armAxis, float length, Axis rotAxis) : armSegment { vector(armAxis, length) }, rotAxis { rotAxis } {}
    };

    template <size_t size, size_t nwrist> // nwrist is the number of joints that form the wrist.
    class Arm {
        static_assert(size >= nwrist, "There needs to be at least as many joints in the arm as wrist joints.");

        std::array<Joint, size> joints;
        Orientation base;

        public:
            static constexpr Orientation ORIGIN { ZERO, IDENTITY };
            static constexpr float DEFAULT_GRANULARITY { 0.001 };

            Arm(std::array<Joint, size> joints, Orientation base = ORIGIN) : joints { joints }, base { base } {}

            // Retrieve the angle of every joint for dispatch to the arm's microcontroller.
            std::array<float, size> collectAngles() {
                std::array<float, size> result {};

                for (int i {}; const Joint& joint : joints) {
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

                    for (int i {}; i < 1'000'000; ++i) { // To avoid an infinite loop with while (true)
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
