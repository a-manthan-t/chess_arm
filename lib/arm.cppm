module;

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

export module arm;

import quaternion;
import path;

export namespace arm {
    using namespace path;
    using namespace quaternion;

    struct Joint {
        const Quaternion armSegment; // The position vector to the arm segment tip from the local origin when angle == 0.
        const Axis rotAxis;
        float angle {};

        Joint(Axis armAxis, float length, Axis rotAxis) : armSegment(vector(armAxis, length)), rotAxis(rotAxis) {}
    };

    class Arm {
        std::vector<Joint> joints;
        size_t wristSize;
        unsigned int delay_ms;
        float granularity;
        Orientation base;

        std::mutex armMutex;
        std::condition_variable flagCondition;
        std::deque<Checkpoint> checkpoints;

        void dispatchAngles() const;
        Path createPath();

        public:
            Arm(const std::vector<Joint>& joints, size_t wristSize, unsigned int delay_ms = 5,
                float granularity = 0.001, const Orientation &base = ORIGIN);

            std::atomic_bool moving {};

            Orientation locateEndEffector() const;
            float errorTo(const Orientation& target) const;
            void ccdTo(const Orientation& target);

            void addCheckpoint(const Checkpoint& checkpoint);
            [[noreturn]] void follow();
    };
}
