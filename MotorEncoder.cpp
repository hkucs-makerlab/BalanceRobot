/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotorEncoder.cpp
 * Author: user
 * 
 * Created on March 22, 2017, 7:38 PM
 */

#include "MotorEncoder.hpp"

MotorEncoder::MotorEncoder() {
}

MotorEncoder::MotorEncoder(const MotorEncoder& orig) {
}

MotorEncoder::MotorEncoder(int pwmGpio, int a1_gpio, int a2_gpio, int outputA_gpio, int outputB_gpio) :
MotorDriver(pwmGpio, a1_gpio, a2_gpio),
outputA(outputA_gpio), outputB(outputB_gpio), counter(0) {
    outputA.inputMode();
    outputB.inputMode();
    lastState = outputA.getState();
}

long MotorEncoder::readCounter() {
    state = outputA.getState();
    if (state != lastState) {
        if (outputB.getState() != state) {
            counter++;
        } else {
            counter--;
        }
    }
    lastState = state;
    return counter;
}

MotorEncoder::~MotorEncoder() {
}

