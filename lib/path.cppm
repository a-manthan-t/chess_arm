module;

#include <vector>

export module path;

import quaternion;

export namespace path {
    using namespace quaternion;

    struct Path {
        Quaternion start, target;
        float startSpeed, endSpeed;

        Path(Quaternion start, Quaternion target, float startSpeed, float endSpeed)
            : start(start), target(target), startSpeed(startSpeed), endSpeed(endSpeed) {}

        virtual Quaternion operator()(float t) const = 0;
        virtual ~Path() = default;

        float length(int segments) const;
    };

    float mergedLength(const std::vector<Path>& paths);
    Quaternion slerp(const Quaternion& startRotation, const Quaternion& targetRotation, float t);

    class Line : Path {
        Line(Quaternion start, Quaternion target, float startSpeed, float endSpeed);
        Quaternion operator()(float t) const override;
    };

    class Circle : Path {
        Quaternion centre { ZERO }, u { ZERO }, v { ZERO };
        float alpha {}, radius {};
        bool useLine {};

        Circle(Quaternion start, Quaternion target, Quaternion passThrough, float startSpeed, float endSpeed);
        Quaternion operator()(float t) const override;
    };

    class CubicBezier : Path {
        Quaternion control1 { ZERO }, control2 { ZERO };

        public:
            CubicBezier(Orientation start, Orientation target, float startSpeed, float endSpeed);
            Quaternion operator()(float t) const override;
    };

    struct Checkpoint {
        Orientation orientation;
        float speed;

        Checkpoint(Orientation orientation, float speed) : orientation(orientation), speed(speed) {}
    };

    std::vector<Path> autoPath(std::vector<Checkpoint>& checkpoints);
}
