/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotorDriver.cpp
 * Author: user
 * 
 * Created on March 22, 2017, 2:27 PM
 */

#include "MotorDriver.hpp"

MotorDriver::MotorDriver() {

}

MotorDriver::MotorDriver(const MotorDriver& orig) {
}

MotorDriver::MotorDriver(int pwmGpio,
        int a1_gpio, int a2_gpio) : SoftPWM(pwmGpio), a_in1(a1_gpio), a_in2(a2_gpio){  
    setFrequency(1000);
    setDutyCyclePercentage(50);
    a_in1.outputMode();
    a_in2.outputMode();
    stop();
}

void MotorDriver::forward() {
    a_in1.setHigh();
    a_in2.setLow();
}

void MotorDriver::backward() {
    a_in1.setLow();
    a_in2.setHigh();
}

void MotorDriver::stop() {
    a_in1.setLow();
    a_in2.setLow();
}

MotorDriver::~MotorDriver() {
}

