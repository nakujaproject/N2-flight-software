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

float q = 0.0001;

float T = 0.1;

float delta_time = 0.1;

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
BLA::Matrix<3, 3> Q = {q, q, q,
                       q, q, q,
                       q, q, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_hat = {BASE_ALTITUDE,
                           0.0,
                           0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};

// kalmanUpdate This filteres our altitude and acceleration values
struct FilteredValues kalmanUpdate(float altitude, float acceleration)
{
    struct FilteredValues return_val;

    // Measurement matrix
    // We are subtracting 9.8 which is gravitational acceleration
    BLA::Matrix<2, 1> Z = {altitude,
                           acceleration - 9.8};
    // Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;
    // Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;
    // Kalman gain
    BLA::Matrix<2, 2> con = (H * P_minus * (~H) + R);
    BLA::Matrix<3, 2> K = P_minus * (~H) * Invert(con);
    // Measurement residual
    Y = Z - (H * x_hat_minus);
    // Updated state estimate
    x_hat = x_hat_minus + K * Y;
    // Updated estimate covariance
    P = (I - K * H) * P_minus;
    Y = Z - (H * x_hat_minus);

    return_val.displacement = x_hat(0);
    return_val.velocity = x_hat(1);
    return_val.acceleration = x_hat(2);

    return return_val;
}

struct FilteredValues UpdatedkalmanUpdate(float altitude, float acceleration)
{
    struct FilteredValues return_val;

    // Measurement matrix
    // For acceleration, we need to subtract the gravitational acceleration constant g
    // from each accelerometer measurement
    BLA::Matrix<2, 1> Z = {altitude,
                           acceleration};
    // PREDICT
    // State prediction - Predict where we're going to be
    BLA::Matrix<3, 1> x_predicted = A * x_previous;
    // Covariance prediction - Predict how much error
    BLA::Matrix<3, 3> p_predicted = (A * p_previous * (~A)) + Q;
    
    // UPDATE
    // Innovation - Prefit - Compare reality against prediction
    Y = Z - (H * x_predicted);
    // Innovation Covariance - Compare real error against prediction
    BLA::Matrix<2, 2> S = (H * p_predicted * (~H)) + R;
    // Kalman gain - Moderate the prediction
    BLA::Matrix<3, 2> K = p_predicted * (~H) * Invert(S);
    // Updated state estimate - New estimate of where we are
    x_previous = x_predicted + (K * Y);
    // Covariance Update - New estimate of error
    p_previous = (I - (K * H)) * p_predicted;
    // Innovation - Postfit - Compare reality against prediction
    Y = Z - (H * x_predicted);

    return_val.displacement = x_previous(0);
    return_val.velocity = x_previous(1);
    return_val.acceleration = x_previous(2);

    return return_val;
}

#endif