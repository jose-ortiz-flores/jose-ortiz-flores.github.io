#include "mex.h"
#include "BDotController.hpp"

// Keep the controller state between MATLAB calls
static BDotController controller;

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // Check inputs: B vector and dt
    if (nrhs != 2)
        mexErrMsgTxt("Two inputs required: magneticField and dt.");

    // Read magnetic field from MATLAB
    double *B = mxGetPr(prhs[0]);
    double dt = mxGetScalar(prhs[1]);

    BDotController::Vector3 magneticField =
        {B[0], B[1], B[2]};

    // Run C++ B-dot controller
    BDotController::Vector3 command =
        controller.update(magneticField, dt);

    // Create MATLAB output vector
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);

    double *output = mxGetPr(plhs[0]);

    output[0] = command[0];
    output[1] = command[1];
    output[2] = command[2];
}