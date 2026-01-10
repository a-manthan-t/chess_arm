#include <print>

import quaternion;
import arm;

using namespace quaternion;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main() {
    std::println("{}", zero.show());
    std::println("{}", (identity + identity).show());
    std::println("{}", rotation(0.5, 0.5, 0.5).toEuler().show());
    std::println("{}", rotation(0.5, 0.5, 0.5).show());
    std::println("{}", rotation(0.5, 0, 0).show());
    using enum Axis;
    std::println("{}", arm::Arm<3> {
        {
            arm::Joint {X, 2, Y},
            arm::Joint {Y, 2, X},
            arm::Joint {Z, 5, X}
        }
    }.show());
}
#endif
