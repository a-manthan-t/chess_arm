module;

#include <vector>

export module arm;

import quaternion;
import path;

export namespace arm {
    using namespace quaternion;

    struct Joint {
        const Quaternion armSegment; // The position vector to the arm segment tip from the local origin when angle == 0.
        const Axis rotAxis;
        float angle {};

        Joint(Axis armAxis, float length, Axis rotAxis) : armSegment(vector(armAxis, length)), rotAxis(rotAxis) {}
    };

    class Arm {
        std::vector<Joint> joints;
        size_t wrist_size;
        Orientation base;

        public:
            static constexpr float DEFAULT_GRANULARITY { 0.001 };

            // Ensure wrist_size <= joints.size()!
            Arm(std::vector<Joint> joints, size_t wrist_size, Orientation base = ORIGIN)
                : joints(std::move(joints)), wrist_size(wrist_size), base(base) {}

            std::vector<float> collectAngles() const;

            Orientation locateEndEffector() const;
            float errorTo(const Orientation& target) const;
            void ccdTo(const Orientation& target, float granularity);

            void follow(const path::Path& path); // const???
            void follow(const std::vector<path::Path>& path);
    };
}
