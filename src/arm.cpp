module;

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fcntl.h>
#include <limits>
#include <mutex>
#include <numbers>
#include <print>
#include <ranges>
#include <termios.h>
#include <unistd.h>
#include <vector>

#ifdef TESTING
#include "../lib/doctest.h"
#endif

// Handles forward, inverse kinematics as well as path following for robot arms.
module arm;

import quaternion;
import path;

namespace arm {
    using namespace path;
    using namespace quaternion;

    // Open connection to the microcontroller over USB and initialise.
    Arm::Arm(const char* device, const std::vector<Joint>& joints, size_t wristSize, const Orientation& base,
        unsigned int delay_ms, float granularity)
        : port(open(device, O_RDWR)), joints(joints), wristSize(wristSize), delay_ms(delay_ms), granularity(granularity), base(base) {
        checkpoints.emplace_front(locateEndEffector(), 0);

        if (port < 0) {
            std::println(stderr, "Could not open device.");
            return;
        }

        termios config {};

        if (tcgetattr(port, &config) != 0) { // Copy over existing config.
            std::println(stderr, "Could not open device.");
            port = -1;
            return;
        }

        cfsetispeed(&config, B115200);
        cfsetospeed(&config, B115200);     // Set baud rate to 115200 (what the microcontroller will be set to).
        tcsetattr(port, TCSANOW, &config); // Apply changes now.
    }

    // Close USB connection once done.
    Arm::~Arm() {
        if (port >= 0) {
            close(port);
        }
    }

    // Retrieve the angle of every joint and dispatch to the arm's microcontroller.
    void Arm::dispatchAngles() const {
        if (port < 0) return;

        std::vector<float> result;
        result.reserve(joints.size());

        for (const Joint& joint : joints) {
            result.push_back(joint.angle);
        }

        write(port, result.data(), sizeof(float) * result.size());
    }

    /* Forward kinematics. */

    // Find the global orientation of the end effector.
    Orientation Arm::locateEndEffector() const {
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
    float Arm::errorTo(const Orientation& target) const {
        auto [position, rotation] { locateEndEffector() };

        return (position - target.position).magnitude() / position.magnitude()
             + std::acosf(dot(rotation, target.rotation)) / std::numbers::pi_v<float>;
    }

    // Use CCD to set the arm's angles to those which bring the end effector as close as possible to the
    // target orientation (local minimum). The wrist is adjusted separately from the rest of the arm.
    void Arm::ccdTo(const Orientation& target) {
        auto adjust = [this, &target](auto joints) -> void {
            float lastError { std::numeric_limits<float>::infinity() };

            for (int i {}; i < 1'000'000; ++i) { // To avoid an infinite loop with while (true)
                for (Joint& joint : joints) {
                    float error { errorTo(target) };
                    joint.angle += granularity;

                    if (errorTo(target) > error) {
                        joint.angle -= 2 * granularity;
                    }
                }

                // Stop if function has converged (i.e., consecutive errors are withing 10^-6 of each other -
                // this is used instead of the conventional proximity to the target.
                if (float error { errorTo(target) }; lastError - error < 0.000'001) {
                    break;
                } else {
                    lastError = error;
                }
            }
        };

        adjust(joints | std::views::take(joints.size() - wristSize));
        adjust(joints | std::views::drop(joints.size() - wristSize));
    }

    /* Path following. */

    // Add a checkpoint to the queue.
    void Arm::addCheckpoint(const Checkpoint& checkpoint) {
        {
            std::lock_guard lock { armMutex };
            checkpoints.push_back(checkpoint);
        }

        if (!moving) {
            moving = true;
            flagCondition.notify_one();
        }
    }

    // Get latest checkpoint.
    Checkpoint Arm::getLatestCheckpoint() {
        std::lock_guard lock { armMutex };
        return checkpoints.front();
    }

    // Craft a path using the next two checkpoints.
    Path Arm::createPath() {
        std::unique_lock lock { armMutex };

        // Stop if we run out of new checkpoints (first is current position) until allowed to move.
        createCheckpoint:
        if (checkpoints.size() == 1) {
            moving = false;
            flagCondition.wait(lock, [&] -> bool { return moving; });
        }

        Checkpoint current { checkpoints.front() };
        checkpoints.pop_front();

        if (current == checkpoints.front()) {
            // TODO toggle grip
            goto createCheckpoint;
        }

        return { current, checkpoints.front() };
    }

    // Generate paths based on the queue of checkpoints and set the arm along those paths.
    [[noreturn]] void Arm::follow() {
        using namespace std::chrono;

        while (true) {
            Path path { createPath() };
            auto startTime = high_resolution_clock::now();

            float t {};
            while (t < 1) {
                ccdTo(path(t));
                dispatchAngles();

                t = static_cast<float>(duration_cast<milliseconds>(high_resolution_clock::now() - startTime).count()) / path.duration;

                std::unique_lock lock { armMutex };
                flagCondition.wait_for(lock, milliseconds(delay_ms), [] -> bool { return false; });
            }

            // Final correction since t < 1, but path ends at t = 1.
            ccdTo(path(1));
            dispatchAngles();
        }
    }
}

#ifdef TESTING
TEST_SUITE("Arm Tests") {
    // Tests involving CCD are of course dependent on the target being reachable with the degrees of freedom given.
    // Watch out for gimbal lock cases during testing (different Euler angle representations possible).

    using namespace quaternion;
    using namespace arm;
    using std::numbers::pi;

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

    Orientation target1 { vector(5, 1.3, 6), rotation(pi/3, pi/6, pi/4) };
    Orientation target2 { vector(2, 3, 4), rotation(0, pi/9, pi/2.5) };
    Orientation target3 { vector(3, -2, 1), rotation(pi/8, pi/5, pi/7) };

    Arm arm { joints, 3 };

    TEST_CASE("Dispatch angles") {
        //
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
        Arm _arm { joints, 3 }; // Local copy since it gets updated.

        _arm.ccdTo(target1);
        CHECK(_arm.errorTo(target1) < 0.0025);

        _arm.ccdTo(target2);
        CHECK(_arm.errorTo(target2) < 0.0025);

        _arm.ccdTo(target3);
        CHECK(_arm.errorTo(target3) < 0.0025);
    }
}
#endif
