module;

#include <memory>
#include <vector>

export module path;

import quaternion;

export namespace path {
    using namespace quaternion;

    struct Checkpoint {
        Orientation orientation;
        float speed;

        Checkpoint(const Orientation &orientation, float speed) : orientation(orientation), speed(speed) {}
    };

    struct Path {
        Quaternion start, target;
        float startSpeed, endSpeed;

        Path(Quaternion start, Quaternion target, float startSpeed, float endSpeed)
            : start(start), target(target), startSpeed(startSpeed), endSpeed(endSpeed) {}

        virtual ~Path() = default;

        virtual Quaternion operator()(float t) const = 0;
        float length(int segments) const;
    };

    std::vector<std::unique_ptr<Path>> autoPath(const std::vector<Checkpoint>& checkpoints);
    Quaternion slerp(Quaternion startRotation, Quaternion targetRotation, float t);
    float mergedLength(const std::vector<std::unique_ptr<Path>>& paths);

    struct Line : Path {
        Line(Quaternion start, Quaternion target, float startSpeed, float endSpeed);
        Quaternion operator()(float t) const override;
    };

    struct Circle : Path {
        Quaternion centre { ZERO }, u { ZERO }, v { ZERO };
        float alpha {}, radius {};
        bool useLine {};

        Circle(Quaternion start, Quaternion target, Quaternion passThrough, float startSpeed, float endSpeed);
        Quaternion operator()(float t) const override;
    };

    struct CubicBezier : Path {
        Quaternion control1 { ZERO }, control2 { ZERO };

        CubicBezier(const Orientation& start, const Orientation& target, float startSpeed, float endSpeed);
        Quaternion operator()(float t) const override;
    };
}
