/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PIDClass.hpp
 * Author: user
 *
 * Created on January 30, 2017, 5:26 PM
 */


// PID for rpi blue one
#ifndef PIDCLASS_HPP
#define PIDCLASS_HPP
//Speed PID Configuration
#define PIDS_P_GAIN 0.005f  // 0.005
#define PIDS_I_GAIN 0.0f
#define PIDS_D_GAIN 360.0f
#define PIDS_EMA_SAMPLES 10.0f
#define PIDS_I_LIMIT  1200.0 //Ilimit is before process gain

//Pitch PID Configuration
#define PIDP_P_GAIN 10.0f    // 9.0f for small
#define PIDP_I_GAIN 100.0f  // 110f for small
#define PIDP_D_GAIN 50.0f   // 34.0f for small
#define PIDP_EMA_SAMPLES 2.0f   //2f
#define PIDP_I_LIMIT  10.0 //Ilimit is before process gain  10

typedef struct {
    float *processGain;
    float *integralTime;
    float *derivateTime;

    double error;
    double accumulatedError;
    double differentialError;
    double lastFeedbackReading;

    float *iLimit;

    float *EMAnumberSamples; //Determines the EMAalpha;
} PID_t;

class PIDClass {
public:
    PIDClass();
    PIDClass(const PIDClass& orig);
    double PIDUpdate(double setpoint, double actual_position, double dTmilliseconds, PID_t* pPID);
    void PIDinit(PID_t * pid, float *pgain, float *igain, float *dgain, float *ilimit, float *numsamples);

    virtual ~PIDClass();
private:
    double exponentialMovingAverage(const double value, const double previousEMA, const double alpha);
    void calculateP(const double setpoint, const double actual_position, PID_t* pPID);
    void calculateI(const double setpoint, const double dTmilliseconds, PID_t* pPID);
    void calculateD(const double actual_position, const double dTmilliseconds, PID_t* pPID);
};

#endif /* PIDCLASS_HPP */

