/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LEDClass.cpp
 * Author: user
 * 
 * Created on March 22, 2017, 10:51 AM
 */

#include "LEDClass.hpp"

LEDClass::LEDClass() {
}

LEDClass::LEDClass(const LEDClass& orig) {
}
LEDClass::LEDClass(int gpio) : SoftPWM(gpio) {
    setFrequency(1000);
    setDutyCyclePercentage(50);
}

LEDClass::~LEDClass() {
}

