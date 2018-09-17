/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GPIOClass.hpp
 * Author: user
 *
 * Created on November 12, 2016, 4:07 PM
 */

#ifndef GPIOCLASS_HPP
#define GPIOCLASS_HPP

class GPIOClass {
public:
    GPIOClass();
    GPIOClass(const GPIOClass& orig);
    GPIOClass(int gpio);
    void inputMode();
    void outputMode();
    int setHigh();
    int setLow();
    int toggle();
    int getState();
    virtual ~GPIOClass();
private:
    int fd, gpio;
    int mode;
    volatile int state;

};

#endif /* GPIOCLASS_HPP */

