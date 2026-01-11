module;

#include <string>

export module quaternion;

export namespace quaternion {
    enum class Axis : char { X, Y, Z };

    struct Quaternion {
        float w, x, y, z;

        // Prefer using the `vector` and `rotation` functions instead to ensure w = 0
        // or magnitude = 1 for other functions.
        constexpr Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

        Quaternion inverse() const;
        Quaternion sign() const;
        float magnitudeSquared() const;
        float magnitude() const;

        std::string show() const;
        Quaternion toEuler(bool degrees) const;
    };
    
    struct Orientation {
        Quaternion position, rotation;

        constexpr Orientation(Quaternion position, Quaternion rotation) : position(position), rotation(rotation) {}
    };

    constexpr Quaternion ZERO { 0, 0, 0, 0 };
    constexpr Quaternion IDENTITY { 1, 0, 0, 0 };
    constexpr Orientation ORIGIN { ZERO, IDENTITY };

    bool operator==(Quaternion q, Quaternion r);
    Quaternion operator+(Quaternion q, Quaternion r);
    Quaternion operator-(Quaternion q, Quaternion r);
    Quaternion operator*(Quaternion q, Quaternion r);
    Quaternion operator*(Quaternion q, float x);
    Quaternion operator/(Quaternion q, float x);

    float dot(Quaternion q, Quaternion r);
    Quaternion cross(Quaternion q, Quaternion r);
    Quaternion apply(Quaternion q, Quaternion v);

    Quaternion vector(Axis axis, float length);
    Quaternion vector(float x, float y, float z);
    Quaternion rotation(Axis axis, float angle);
    Quaternion rotation(float x, float y, float z);
}
