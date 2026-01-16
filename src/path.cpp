module;

#include <cmath>

#ifdef TESTING
#include "../lib/doctest.h"
#endif

// Handles calculation of targets at different fractions along a cubic bezier path.
module path;

import quaternion;

namespace path {
    using namespace quaternion;

    // Where segments is the number of segments the path should be broken into
    // for calculating its length.
    Path::Path(const Checkpoint& start, const Checkpoint& target, unsigned int segments)
        : start(start), target(target) {
        float distance = (target.orientation.position - start.orientation.position).magnitude() / 3;

        control1 = start.orientation.position + start.orientation.rotation * distance;
        control1.w = 0; // Since the rotation will have some w, which we don't want in the control vector.
        control2 = target.orientation.position - target.orientation.rotation * distance;
        control2.w = 0;

        Quaternion previous { start.orientation.position };

        for (int i { 1 }; i <= segments; ++i) {
            Quaternion temp { (*this)(static_cast<float>(i) / static_cast<float>(segments)).position };
            length += (temp - previous).magnitude();
            previous = temp;
        }

        // scaled path length = (v + w)/2 due to symmetry, *1000 for s -> ms
        duration = 2'000 * length / (start.speed + target.speed);
    }

    // Calculate the new orientation of the robot a fraction t along its path duration.
    Orientation Path::operator()(float t) const {
        // Please see the bottom of this file (before the tests) for an explanation of this lambda.
        float pt = [this](float t) -> float { // Fraction travelled along path.
            float ttt = t * t * t; // To avoid pow.
            return 2.f * (start.speed * t + (target.speed - start.speed) * (ttt - 0.5f * ttt * t))
                 / (start.speed + target.speed);
        }(t);

        float ptt { pt * pt }, ptm1tm1 { (pt - 1) * (pt - 1) }; // To avoid using pow.
        Quaternion newPosition {
            start.orientation.position * ptm1tm1 * (1 - pt)
            + control1 * (3 * ptm1tm1 * pt)
            + control2 * (3 * ptt * (1 - pt))
            + target.orientation.position * ptt * pt
        };

        float dotProduct { dot(start.orientation.rotation, target.orientation.rotation) };
        if (dotProduct < -0.999 || dotProduct > 0.999) {
            // Linear interpolation due to instability as denominator in formula below `if` statement approaches 0.
            return { newPosition, start.orientation.rotation * (1 - pt) + target.orientation.rotation * pt };
        }

        float angle { std::acosf(dotProduct) }, sinAngle { std::sinf(angle) };
        Quaternion newRotation { // Calculated using SLERP formula.
            start.orientation.rotation * (std::sinf(angle * (1 - pt)) / sinAngle)
            + target.orientation.rotation * (std::sinf(angle * pt) / sinAngle)
        };

        return { newPosition, newRotation };
    }
}

/*
 *
 * Explanation of the lambda:
 *
 * We need to calculate the fraction of the path travelled pt, given t - the fraction of the path's duration that has
 * elapsed - and start and end speeds between which we assume a cubic speed profile. At time x:
 *
 * speed = v + (3x^2 - 2x^3)(w - v)       simplified from the cubic bezier formula with v and w as control points.
 * distance = int_0^t (speed) dx = [vx + (x^3-0.5x^4)(w-v)]_0^t
 *                               = vt + (w-v)(t^3-0.5t^4)
 * This needs to be scaled according to the speeds: when t = 1 (full path travelled), distance = (v + w) / 2,
 * which is used as the denominator here.
 *
 */

#ifdef TESTING
TEST_SUITE("Path Planning Tests") {

}
#endif
