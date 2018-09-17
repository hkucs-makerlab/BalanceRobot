/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SoftPWM.cpp
 * Author: user
 * 
 * Created on March 21, 2017, 9:24 PM
 */

#include <cstdio>
#include <cstdlib>
#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef __cplusplus
}
#endif
#include "SoftPWM.hpp"

SoftPWM::SoftPWM() : gpio(-1) {
}

SoftPWM::SoftPWM(const SoftPWM& orig) {
}

SoftPWM::SoftPWM(int gpio) : gpio(-1) {
    char stringbuffer[64];
    char pwmClassAttr[64];
    this->gpio = gpio;

    snprintf(stringbuffer, sizeof (stringbuffer), "/sys/class/soft_pwm");
    //std::cout << stringbuffer << std::endl;
    if (access(stringbuffer, F_OK) != 0) {
        std::cout << "soft_pwm kernel module not loaded,abort!";
        exit(-1);
    }

    snprintf(pwmClassAttr, sizeof (pwmClassAttr), "/sys/class/soft_pwm/pwm-%d", gpio);
    //std::cout << pwmClassAttr << std::endl;
    if (access(pwmClassAttr, F_OK) != 0) {
        fd = open("/sys/class/soft_pwm/export", O_WRONLY | O_NONBLOCK);
        if (fd < 0) {
            std::cout << "failed to open export attribute!\n";
            exit(-2);
        }
        sprintf(stringbuffer, "%d\n", gpio);
        write(fd, stringbuffer, strlen(stringbuffer));
        close(fd);
    }

    snprintf(stringbuffer, sizeof (stringbuffer), "%s/%s", pwmClassAttr, "period_ns");
    //std::cout << stringbuffer << std::endl;
    period_fd = open(stringbuffer, O_RDWR | O_NONBLOCK);
    if (period_fd < 0) {
        std::cout << "failed to open period_ns attribute!\n";
        exit(-3);
    }
    snprintf(stringbuffer, sizeof (stringbuffer), "%s/%s", pwmClassAttr, "duty_cycle_ns");
    //std::cout << stringbuffer << std::endl;
    duty_cycle_fd = open(stringbuffer, O_RDWR | O_NONBLOCK);
    if (period_fd < 0) {
        std::cout << "failed to open period_ns attribute!\n";
        exit(-4);
    }
}

int SoftPWM::setDutyCyclePercentage(float range) {
    if (range < 0 || range > 100) return -1;
    float duty_cycle = (float) period * range / 100;
    //std::cout << "range " << range << " duty cycle " <<  (unsigned long)duty_cycle << std::endl;
    setDutyCycle((unsigned long)duty_cycle);
    return 0;
}

void SoftPWM::setDutyCycle(unsigned long duty_cycle) {
    char stringbuffer[64];
    snprintf(stringbuffer, sizeof (stringbuffer), "%lu", duty_cycle);
    write(duty_cycle_fd, stringbuffer, strlen(stringbuffer));
    this->duty_cycle = duty_cycle;
    //std::cout << "duty cycle " << stringbuffer << std::endl;
}

void SoftPWM::setPeroid(unsigned long period) {
    char stringbuffer[64];
    snprintf(stringbuffer, sizeof (stringbuffer), "%lu", period);
    write(period_fd, stringbuffer, strlen(stringbuffer));
    this->period = period;
    //std::cout << "peroid " << stringbuffer << std::endl;
}

void SoftPWM::setFrequency(unsigned long freq) {
    unsigned long peroid = 1000000000 / freq;
    setPeroid(peroid);
}

void SoftPWM::delay(int ms) {
    usleep(ms * 1000);
}

SoftPWM::~SoftPWM() {
    if (period_fd != -1) close(period_fd);
    if (duty_cycle_fd != 1)close(duty_cycle_fd);
    fd = open("/sys/class/soft_pwm/unexport", O_WRONLY | O_NONBLOCK);
    if (fd >= 0) {
        char stringbuffer[8];
        snprintf(stringbuffer, sizeof (stringbuffer), "%d", gpio);
        write(fd, stringbuffer, strlen(stringbuffer));
        close(fd);
    }

}
