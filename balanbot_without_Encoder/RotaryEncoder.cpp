/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RotaryEncoder.cpp
 * Author: user
 * 
 * Created on April 8, 2017, 5:12 PM
 */

#include "RotaryEncoder.hpp"

RotaryEncoder::RotaryEncoder() {
}

RotaryEncoder::RotaryEncoder(const RotaryEncoder& orig) {
}

RotaryEncoder::RotaryEncoder(int outputA_gpio, int outputB_gpio) : outputA(outputA_gpio), outputB(outputB_gpio), counter(0) {
    outputA.inputMode();
    outputB.inputMode();
    lastState = outputA.getState();
}

long RotaryEncoder::readCounter() {
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

long RotaryEncoder::reSet(int value){
    counter = value;
}

RotaryEncoder::~RotaryEncoder() {
}

