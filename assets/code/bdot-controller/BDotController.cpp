#include "BDotController.hpp"

BDotController::BDotController()
    : previousField_{0.0, 0.0, 0.0},
      initialized_(false)
{
}

BDotController::Vector3 BDotController::update(
    const Vector3& magneticField, double dt)
{
    // GST-600 Mk2 maximum dipole moments [A m^2]
    const Vector3 maxDipole = {0.40, 0.40, 0.50};

    // No B-dot estimate is available at the first measurement.
    if(!initialized_)
    {
       previousField_ = magneticField;
       initialized_ = true;

       return{0.0, 0.0, 0.0};
    }

    Vector3 command = {0.0, 0.0, 0.0};



    for (int i = 0; i < 3; ++i)
    {
        double Bdot =
            (magneticField[i] - previousField_[i]) / dt;

        if (Bdot > 0.0)
            command[i] = -maxDipole[i];
        else if (Bdot < 0.0)
            command[i] = maxDipole[i];
    }

    previousField_ = magneticField;

    return command;
}