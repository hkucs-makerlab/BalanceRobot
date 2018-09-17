/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.cpp
 * Author: user
 * 
 * Created on January 30, 2017, 5:18 PM
 */

#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::KalmanFilter(const KalmanFilter& orig) {
}

void KalmanFilter::InitKalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_kalmanangle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;

    kalmanangle = 0; // Reset the kalmanangle
    bias = 0; // Reset bias

    P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting kalmanangle (use setkalmanangle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
}
// The kalmanangle should be in degrees and the rate should be in degrees per second and the delta time in seconds

double KalmanFilter::getkalmanangle(double newkalmanangle, double newRate, double dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    kalmanangle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_kalmanangle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    S = P[0][0] + R_measure;
    /* Step 5 */
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate kalmanangle and bias - Update estimate with measurement zk (newkalmanangle)
    /* Step 3 */
    y = newkalmanangle - kalmanangle;
    /* Step 6 */
    kalmanangle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return kalmanangle;
};

KalmanFilter::~KalmanFilter() {
}

