module;

#include <cmath>
#include <numbers>

#ifdef TESTING
#include "../lib/doctest.h"
#endif

// Handles calculation of targets at different fractions along a path.
module path;

import quaternion;

namespace path {
    using namespace quaternion;

    /* Constructors */

    Line::Line(Quaternion start, Quaternion target, float startSpeed, float endSpeed)
        : Path(start, target, startSpeed, endSpeed) {}

    Circle::Circle(Quaternion start, Quaternion target, Quaternion passThrough, float startSpeed, float endSpeed)
        : Path(start, target, startSpeed, endSpeed) {
        Quaternion
            p0p1 { start - passThrough },
            p1p2 { passThrough - target },
            p2p0 { target - start } ,
            normal { cross(p0p1, p1p2) };

        if (float normalMag { normal.magnitude() }; normalMag < 0.000'1) {
            useLine = true;
        } else {
            centre = start + cross(p2p0 * p0p1.magnitudeSquared() + p0p1 * p2p0.magnitudeSquared(), normal)
                / (2 * normal.magnitudeSquared());

            Quaternion
                cp0 { start - centre },
                cp2 { target - centre };

            float sign = cross(cp0, cp2).sign() + normal.sign() == ZERO ? -1.f : 1.f;

            radius = std::sqrtf(p0p1.magnitudeSquared() * p1p2.magnitudeSquared() * p2p0.magnitudeSquared()) / (2 * normalMag);
            alpha = sign * -std::acosf(dot(cp0, cp2) / std::sqrtf(cp0.magnitudeSquared() * cp2.magnitudeSquared()))
                    + (sign == 1.f ? 0.f : -2 * std::numbers::pi_v<float>); // Angle to end at
            u = cp0 / radius;                                               // Normalised vector from centre to start
            v = cross(u, normal / normalMag);                          // Perpendicular vector to u in plane of circle
        }
    }

    CubicBezier::CubicBezier(Orientation start, Orientation target, float startSpeed, float endSpeed)
        : Path(start.position, target.position, startSpeed, endSpeed) {
        float distance = (target.position - start.position).magnitude() / 3;

        // The w component will get discarded, which is fine.
        control1 = start.position + start.rotation * distance;
        control2 = target.position - target.rotation * distance;
    }

    /* Path calculators */

    Quaternion Line::operator()(float t) const {
        return start * (1 - t) + target * t;
    }

    Quaternion Circle::operator()(float t) const {
        return useLine ? start * (1 - t) + target * t
                       : centre + (u * std::cosf(alpha * t) + v * std::sinf(alpha * t)) * radius;
    }

    Quaternion CubicBezier::operator()(float t) const {
        float tt { t * t }, tm1tm1 { (t - 1) * (t - 1) }; // To avoid using pow.

        return start * tm1tm1 * (1 - t)
             + control1 * (3 * tm1tm1 * t)
             + control2 * (3 * tt * (1 - t))
             + target * tt * t;
    }

    // To interpolate the rotation of the end effector.
    Quaternion slerp(const Quaternion& startRotation, const Quaternion& targetRotation, float t) {
        float dotProduct { dot(startRotation, targetRotation) };

        if (dotProduct < -0.999 || dotProduct > 0.999) {
            return startRotation * (1 - t) + targetRotation * t;
        }

        float angle { std::acosf(dotProduct) }, sinAngle { std::sinf(angle) };
        return startRotation * (std::sinf(angle * (1 - t)) / sinAngle)
             + targetRotation * (std::sinf(angle * t) / sinAngle);
    }

    /* Speed control functions. */

    // Calculate a path's length by splitting it into a number of straight
    // line segments and adding their lengths.
    float Path::length(int segments = 1000) const {
        float result {};
        Quaternion previous { start };

        for (int i = 1; i <= segments; ++i) {
            Quaternion temp = (*this)(static_cast<float>(i) / static_cast<float>(segments));
            result += (temp - previous).magnitude();
            previous = temp;
        }

        return result;
    }

    float mergedLength(const std::vector<Path>& paths) {
        float result {};

        for (const Path& path : paths) {
            result += path.length();
        }

        return result;
    }
}

#ifdef TESTING
TEST_SUITE("Path Planning Tests") {
}
#endif
