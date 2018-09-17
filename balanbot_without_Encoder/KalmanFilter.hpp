/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.hpp
 * Author: user
 *
 * Created on January 30, 2017, 5:18 PM
 */

#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

class KalmanFilter {
public:
    KalmanFilter();
    KalmanFilter(const KalmanFilter& orig);
    void InitKalman();
    double getkalmanangle(double newkalmanangle, double newRate, double dt);
    void setkalmanangle(double newkalmanangle) {
        kalmanangle = newkalmanangle;
    }; // Used to set kalmanangle, this should be set as the starting kalmanangle

    double getRate() {
        return rate;
    }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQkalmanangle(double newQ_kalmanangle) {
        Q_kalmanangle = newQ_kalmanangle;
    };

    void setQbias(double newQ_bias) {
        Q_bias = newQ_bias;
    };

    void setRmeasure(double newR_measure) {
        R_measure = newR_measure;
    };

    double getQkalmanangle() {
        return Q_kalmanangle;
    };

    double getQbias() {
        return Q_bias;
    };

    double getRmeasure() {
        return R_measure;
    };

    virtual ~KalmanFilter();
private:
    /* Kalman filter variables */
    double Q_kalmanangle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double kalmanangle; // The kalmanangle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getkalmanangle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // kalmanangle difference
    double S; // Estimate error

};

#endif /* KALMANFILTER_HPP */

