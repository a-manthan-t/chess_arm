module;

#include <cmath>
#include <format>
#include <numbers>
#include <string>

#ifdef TESTING
#include "doctest.h"
#endif

// Handles basic operations related to quaternions as needed in this project.
export module quaternion;

export namespace quaternion {
    enum class Axis : int8_t { X, Y, Z };

    struct Quaternion {
        float w, x, y, z;

        // Prefer using vector and rotation functions to construct instead to ensure w = 0 or the magnitude = 1.
        constexpr Quaternion(float w, float x, float y, float z) : w { w }, x { x }, y { y }, z { z } {}

        /* Functions to calculate quaternion attributes. */

        // Assumes it is called on a unit quaternion (so its inverse equals its conjugate).
        Quaternion inverse() const {
            return { w, -x, -y, -z };
        }

        float magnitudeSquared() const {
            return std::powf(w, 2) + std::powf(x, 2) + std::powf(y, 2) + std::powf(z, 2);
        }

        float magnitude() const {
            return std::sqrtf(magnitudeSquared());
        }

        /* Helpful functions for testing. */

        std::string show() const { // Components displayed to 4 d.p.
            auto r4 = [](float v) { return std::round(v * 10'000) / 10'000; };
            return std::format("(w: {}, x: {}, y: {}, z: {})", r4(w), r4(x), r4(y), r4(z));
        }

        // Extract the angle rotated around each axis from the quaternion, assuming it is not a vector.
        // The returned structure does not actually represent a quaternion.
        Quaternion toEuler(bool degrees = true) const {
            using namespace std;
            const float convert { degrees ? 180/numbers::pi_v<float> : 1 };

            return {
                0,
                atan2f(
                    2 * (w * x + y * z),
                    1 - 2 * (x * x + y * y)
                ) * convert,
                (atan2f(
                    sqrt(1 + 2 * (w * y - x * z)),
                    sqrt(1 - 2 * (w * y - x * z))
                ) * 2 - numbers::pi_v<float>/2) * convert,
                atan2f(
                    2 * (w * z + x * y),
                    1 - 2 * (y * y + z * z)
                ) * convert
            };
        }
    };

    /* Quaternion constants. */

    constexpr Quaternion ZERO { 0, 0, 0, 0 };
    constexpr Quaternion IDENTITY { 1, 0, 0, 0 };

    /* Operations on quaternions. */

    // Checks for equality of the q and r with their components rounded to 5 d.p.
    bool operator==(Quaternion q, Quaternion r) {
        auto r5 = [](float v) -> int { return static_cast<int>(v * std::pow(10, 5)); };
        return r5(q.w) == r5(r.w) && r5(q.x) == r5(r.x) && r5(q.y) == r5(r.y) && r5(q.z) == r5(r.z);
    }

    // Assumes the quaternions are vectors.
    Quaternion operator+(Quaternion q, Quaternion r) {
        return { 0, q.x + r.x, q.y + r.y, q.z + r.z };
    }

    // Assumes the quaternions are vectors.
    Quaternion operator-(Quaternion q, Quaternion r) {
        return { 0, q.x - r.x, q.y - r.y, q.z - r.z };
    }

    // Assumes q and r are unit quaternions.
    Quaternion operator*(Quaternion q, Quaternion r) {
        return {
            q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z,
            q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
            q.w * r.y + q.y * r.w + q.z * r.x - q.x * r.z,
            q.w * r.z + q.z * r.w + q.x * r.y - q.y * r.x
        };
    }

    // Assumes q is a vector.
    Quaternion operator*(Quaternion q, float x) {
        return { 0, q.x * x, q.y * x, q.z * x };
    }

    // Assumes q is a vector.
    Quaternion operator/(Quaternion q, float x) {
        return { 0, q.x / x, q.y / x, q.z / x };
    }

    float dot(Quaternion q, Quaternion r) {
        return q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z;
    }

    // Assumes the quaternions are vectors.
    Quaternion cross(Quaternion q, Quaternion r) {
        return {
            0,
            q.y * r.z - q.z * r.y,
            q.z * r.x - q.x * r.z,
            q.x * r.y - q.y * r.x
        };
    }

    // Rotate v using q, assuming v is a vector and q is a unit quaternion.
    Quaternion apply(Quaternion q, Quaternion v) {
        return q * v * q.inverse();
    }

    /* Functions for creating quaternions. */

    Quaternion vector(Axis axis, float length) {
        return {
            0,
            axis == Axis::X ? length : 0,
            axis == Axis::Y ? length : 0,
            axis == Axis::Z ? length : 0
        };
    }

    Quaternion vector(float x, float y, float z) {
        return { 0, x, y, z };
    }

    Quaternion rotation(Axis axis, float angle) {
        return {
            std::cosf(angle/2),
            axis == Axis::X ? std::sinf(angle/2) : 0,
            axis == Axis::Y ? std::sinf(angle/2) : 0,
            axis == Axis::Z ? std::sinf(angle/2) : 0
        };
    }

    Quaternion rotation(float x = 0, float y = 0, float z = 0) {
        return rotation(Axis::Z, z) * rotation(Axis::Y, y) * rotation(Axis::X, x);
    }
}


#ifdef TESTING
TEST_SUITE("Quaternion Tests") {
    using namespace quaternion;
    using std::numbers::pi;
    float epsilon { std::powf(10, -4) };

    Quaternion
        q { rotation(Axis::X, pi/2) },
        r { rotation(pi/6, pi/4, pi/3) },
        u { vector(Axis::Y, 12) },
        v { vector(2, 4, 3) },
        w { vector(7, 9, -3) };

    TEST_CASE("Construction") {
        SUBCASE("Create vector") {
            CHECK(u.w == 0);
            CHECK(u.x == 0);
            CHECK(u.y == 12);
            CHECK(u.z == 0);

            CHECK(v.w == 0);
            CHECK(v.x == 2);
            CHECK(v.y == 4);
            CHECK(v.z == 3);
        }

        SUBCASE("Create rotation") {
            CHECK(doctest::Approx(q.w).epsilon(epsilon) == 0.70711f);
            CHECK(doctest::Approx(q.x).epsilon(epsilon) == 0.70711f);
            CHECK(doctest::Approx(q.y).epsilon(epsilon) == 0.f);
            CHECK(doctest::Approx(q.z).epsilon(epsilon) == 0.f);

            CHECK(doctest::Approx(r.w).epsilon(epsilon) == 0.82236f);
            CHECK(doctest::Approx(r.x).epsilon(epsilon) == 0.02226f);
            CHECK(doctest::Approx(r.y).epsilon(epsilon) == 0.43968f);
            CHECK(doctest::Approx(r.z).epsilon(epsilon) == 0.36042f);
        }
    }

    TEST_CASE("Attribute calculation") {
        SUBCASE("Inverse") {
            Quaternion qInv { q.inverse() }, rInv { r.inverse() };

            CHECK(qInv.w == q.w);
            CHECK(qInv.x == -q.x);
            CHECK(qInv.y == -q.y);
            CHECK(qInv.z == -q.z);

            CHECK(rInv.w == r.w);
            CHECK(rInv.x == -r.x);
            CHECK(rInv.y == -r.y);
            CHECK(rInv.z == -r.z);
        }

        SUBCASE("Magnitude") {
            CHECK(doctest::Approx(q.magnitude()).epsilon(epsilon) == 1);
            CHECK(doctest::Approx(r.magnitude()).epsilon(epsilon) == 1);
            CHECK(doctest::Approx(u.magnitude()).epsilon(epsilon) == 12);
            CHECK(v.magnitude() == std::sqrtf(29));
        }
    }

    TEST_CASE("Conversion to Euler angles (note gimbal lock rotations can be encoded in multiple ways)") {
        CHECK(q.toEuler().show() == vector(90, 0, 0).show());
        CHECK(r.toEuler().show() == vector(30, 45, 60).show());
        CHECK(q.toEuler(false) == vector(pi/2, 0, 0));
        CHECK(r.toEuler(false) == vector(pi/6, pi/4, pi/3));
    }

    TEST_CASE("Operations") {
        SUBCASE("Equality") {
            CHECK(ZERO == vector(0, 0, 0));
            CHECK(IDENTITY == rotation(0, 0, 0));
        }

        SUBCASE("Addition/Subtraction") {
            CHECK(u + v + w == vector(9, 25, 0));
            CHECK(w - v == vector(5, 5, -6));
        }

        SUBCASE("Multiplication (by quaternion)") {
            CHECK((q * r).show() == "(w: 0.5658, x: 0.5972, y: 0.056, z: 0.5658)");
            CHECK((r * q).show() == "(w: 0.5658, x: 0.5972, y: 0.5658, z: -0.056)");
            CHECK(r * r.inverse() == IDENTITY);
        }

        SUBCASE("Multiplication/Division (by scalar)") {
            CHECK(u * 2.45 == vector(0, 12.f * 2.45f, 0));
            CHECK(v / 0.37 == vector(2 / 0.37, 4 / 0.37, 3 / 0.37));
        }

        SUBCASE("Dot and Cross product") {
            CHECK(dot(v, w) == 41);
            CHECK(cross(v, w) == vector(-39, 27, -10));
        }

        SUBCASE("Apply rotation") {
            CHECK(apply(q, u).show() == "(w: 0, x: 0, y: -0, z: 12)");
            CHECK(apply(r, v).show() == "(w: -0, x: 0.6318, y: 5.0225, z: 1.8371)");
        }
    }
}
#endif
