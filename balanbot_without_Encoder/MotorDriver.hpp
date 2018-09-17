/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotorDriver.hpp
 * Author: user
 *
 * Created on March 22, 2017, 2:27 PM
 */

#ifndef MOTORDRIVER_HPP
#define MOTORDRIVER_HPP
#include "SoftPWM.hpp"
#include "GPIOClass.hpp"

class MotorDriver : public SoftPWM {
public:
    MotorDriver();
    MotorDriver(const MotorDriver& orig);
    MotorDriver(int pwmGpio, int a1_gpio, int a2_gpio);
    void forward();
    void backward();
    void stop();
    virtual ~MotorDriver();
private:
    GPIOClass a_in1, a_in2;
};

#endif /* MOTORDRIVER_HPP */

