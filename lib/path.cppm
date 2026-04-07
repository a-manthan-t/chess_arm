export module path;

import quaternion;

export namespace path {
    using namespace quaternion;

    struct Checkpoint {
        Orientation orientation;
        float speed;

        Checkpoint(const Orientation& orientation, float speed) : orientation(orientation), speed(speed) {}
    };

    bool operator==(const Checkpoint& c, const Checkpoint& d);

    struct Path {
        Checkpoint start, target;
        Quaternion control1 { ZERO }, control2 { ZERO };
        float length {}, duration;

        Path(const Checkpoint& start, const Checkpoint& target, unsigned int segments = 1000);
        Orientation operator()(float t) const;
    };
}
