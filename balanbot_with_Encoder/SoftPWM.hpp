/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SoftPWM.h
 * Author: user
 *
 * Created on March 21, 2017, 9:24 PM
 */

#ifndef SOFTPWM_H
#define SOFTPWM_H

class SoftPWM {
public:
    SoftPWM();
    SoftPWM(int gpio);
    SoftPWM(const SoftPWM& orig);
    void setFrequency(unsigned long);
    void setPeroid(unsigned long);
    void setDutyCycle(unsigned long);
    int setDutyCyclePercentage(float);
    void setEnable(int enable);
    void delay(int ms);
    virtual ~SoftPWM();
protected:
    unsigned long period;
    unsigned long duty_cycle;

private:
    int gpio;
    int fd;
    int period_fd;
    int duty_cycle_fd;
    int enable_fd;

};

#endif /* SOFTPWM_H */

