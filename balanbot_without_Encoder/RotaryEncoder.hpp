/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RotaryEncoder.hpp
 * Author: user
 *
 * Created on April 8, 2017, 5:12 PM
 */

#ifndef ROTARYENCODER_HPP
#define ROTARYENCODER_HPP
#include "GPIOClass.hpp"

class RotaryEncoder {
public:
    RotaryEncoder();
    RotaryEncoder(const RotaryEncoder& orig);
    RotaryEncoder(int outputA_gpio, int outputB_gpio);
    long readCounter();
    long reSet(int value);
   
    virtual ~RotaryEncoder();
private:
    GPIOClass outputA;
    GPIOClass outputB;
;
    volatile long counter;
    int state;
    int lastState;
};

#endif /* ROTARYENCODER_HPP */

