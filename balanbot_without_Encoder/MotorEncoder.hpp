/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotorEncoder.hpp
 * Author: user
 *
 * Created on January 30, 2017, 2:17 PM
 */

#ifndef MOTORENCODER_HPP
#define MOTORENCODER_HPP

#include<cstdio>
#include<cstring>
#include<pthread.h>
#include<stdlib.h>
#include<unistd.h>
#include <mraa.h>

#include "GPIOClass.hpp"

class MotorEncoder {
public:
    MotorEncoder();
    MotorEncoder(const MotorEncoder& orig);
    void initEncoders(int a, int b, int c, int d);
    void EncoderInterruptAA(void * args);
    void EncoderInterruptBB(void * args);
    void ResetEncoders();
    double GetEncoder();
    void GetEncoders(double * temp);
    void GetEncoderChange(double * temp);
    void EncoderAddPos2(double distance1, double distance2);
    void EncoderAddPos(double distance);
    void CloseEncoder();
    virtual ~MotorEncoder();
private:
    friend void EncoderInterrupt(void * args);
    GPIOClass gpio[4];
    mraa_gpio_context encoderx[ 4 ];
    int lastpins[ 4 ];
    volatile double position[ 2 ];
};

#endif /* MOTORENCODER_HPP */

