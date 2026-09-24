#pragma once

#include <array>

class BDotController
{
public:
    using Vector3 = std::array<double, 3>;

    BDotController();

    Vector3 update(const Vector3& magneticField, double dt);

private:
    Vector3 previousField_;
    bool initialized_;
};