#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <BasicLinearAlgebra.h>
#include "defs.h"

// Kalman filtering, also known as linear quadratic estimation (LQE),
// is an algorithm that uses a series of measurements observed over time,
// including statistical noise and other inaccuracies,
// to produce estimates of unknown variables that are more accurate 
// than those based on a single measurement alone,
// by estimating a joint probability distribution over the variables for each timeframe.

using namespace BLA;

float q = 0.001;

float T = 0.1;

// This should be calculates automatically
// as it is the time between kalman update
float delta_time = 0.05;

float delta_time_squared = (delta_time*delta_time)/2;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, delta_time, delta_time_squared,
                       0, 1.0, delta_time,
                       0, 0, 1.0};
// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                       0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

// Measurement error covariance
BLA::Matrix<2, 2> R = {0.25, 0,
                       0, 0.75};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, 0.0, 0.0,
                       0.0, q, 0.0,
                       0.0, 0.0, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_previous = {BASE_ALTITUDE,
                           0.0,
                           0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};

// kalmanUpdate This filteres our altitude and acceleration values
struct FilteredValues kalmanUpdate(float altitude, float acceleration)
{
    struct FilteredValues return_val;

    // Measurement matrix
    // For acceleration, we need to subtract the gravitational acceleration constant g
    // from each accelerometer measurement
    BLA::Matrix<2, 1> Z = {altitude,
                           acceleration - 9.8};
    
    // PREDICT
    // State prediction - Predict where we're going to be
    BLA::Matrix<3, 1> x_predicted = A * x_previous;
    // Covariance prediction - Predict how much error
    BLA::Matrix<3, 3> p_predicted = ((A * P) * (~A)) + Q;
    
    // UPDATE
    // Innovation - Prefit - Compare reality against prediction
    Y = Z - (H * x_predicted);
    // Innovation Covariance - Compare real error against prediction
    BLA::Matrix<2, 2> S = ((H * p_predicted) * (~H)) + R;
    // Kalman gain - Moderate the prediction
    BLA::Matrix<3, 2> K = p_predicted * (~H) * Invert(S);

    // Updated state estimate - New estimate of where we are
    x_previous = x_predicted + (K * Y);
    // Covariance Update - New estimate of error
    P = (I - (K * H)) * p_predicted;
    // Innovation - Postfit - Compare reality against prediction
    Y = Z - (H * x_predicted);

    return_val.displacement = x_previous(0);
    return_val.velocity = x_previous(1);
    return_val.acceleration = x_previous(2);

    return return_val;
}

#endif