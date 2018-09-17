/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GPIOClass.cpp
 * Author: user
 * 
 * Created on November 12, 2016, 4:07 PM
 */

#include <cstdio>
#include <cstring>
#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>
#include <sys/time.h>
#include <stdlib.h>
#ifdef __cplusplus
}
#endif
#include "GPIOClass.hpp"

#define GPIO_UNDEF_MODE 0
#define GPIO_INPUT_MODE  1
#define GPIO_OUTPUT_MODE 2

GPIOClass::GPIOClass() : fd(-1), gpio(-1), mode(GPIO_UNDEF_MODE), state(-1) {
}

GPIOClass::GPIOClass(const GPIOClass& orig) {
}

GPIOClass::GPIOClass(int gpio) : fd(-1), gpio(-1), mode(GPIO_UNDEF_MODE) {
    char stringbuffer[64];
    this->gpio = gpio;
    sprintf(stringbuffer, "/sys/class/gpio/gpio%d/value", gpio);
    if (access(stringbuffer, F_OK) == 0) {
        syslog(LOG_NOTICE, "%s exists!", stringbuffer);
        goto __init_gpio;
    }

    fd = open("/sys/class/gpio/export", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        syslog(LOG_NOTICE, "open %s failed!", stringbuffer);
        exit(-1);
    }
    sprintf(stringbuffer, "%d\n", gpio);
    write(fd, stringbuffer, strlen(stringbuffer));
    close(fd);
    //

__init_gpio:
    sprintf(stringbuffer, "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(stringbuffer, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        syslog(LOG_NOTICE, "open %s failed!", stringbuffer);
        exit(-2);
    }
    syslog(LOG_NOTICE, "open %s success!", stringbuffer);
}

void GPIOClass::inputMode() {
    char stringbuffer[64];
    sprintf(stringbuffer, "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(stringbuffer, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        syslog(LOG_NOTICE, "open %s failed!", stringbuffer);
        exit(-3);
    }
    write(fd, "in", 2);
    close(fd);
    mode = GPIO_INPUT_MODE;
    state=-1;
}

void GPIOClass::outputMode() {
    char stringbuffer[64];
    sprintf(stringbuffer, "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(stringbuffer, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        syslog(LOG_NOTICE, "open %s failed!", stringbuffer);
        exit(-4);
    }
    write(fd, "out", 3);
    close(fd);
    mode = GPIO_OUTPUT_MODE;
    state=-1;
}

int GPIOClass::setLow() {
    if (mode != GPIO_OUTPUT_MODE) return -1;
    if (state == 0) return 1;
    lseek(fd, 0, SEEK_SET);
    state = 0;
    return write(fd, "0", 1);
}

int GPIOClass::setHigh() {
    if (mode != GPIO_OUTPUT_MODE) return -1;
    if (state == 1) return 1;
    lseek(fd, 0, SEEK_SET);
    state = 1;
    return write(fd, "1", 1);
}

int GPIOClass::toggle() {
    if (mode != GPIO_OUTPUT_MODE || state == -1) return -1;
    lseek(fd, 0, SEEK_SET);    
    state = !state;
    char ch = '0'+state;
    return write(fd, &ch, 1);
}

int GPIOClass::getState() {
    if (mode != GPIO_INPUT_MODE) return -1;
    char buf[4];
    lseek(fd, 0, SEEK_SET);
    read(fd, buf, sizeof (buf));
    state = buf[0] - '0';
    return state;
}

GPIOClass::~GPIOClass() {
    if (fd < 0) return;
    close(fd);

    fd = open("/sys/class/gpio/unexport", O_WRONLY | O_NONBLOCK);
    if (fd >= 0) {
        char stringbuffer[8];
        sprintf(stringbuffer, "%d", gpio);
        write(fd, stringbuffer, strlen(stringbuffer));
        close(fd);
    }
}

static void gpioExample() {
    GPIOClass gpio(26);

    // for output
    gpio.outputMode();
    gpio.setHigh();
    gpio.setLow();
    gpio.toggle();

    // for input
    gpio.inputMode();
    int state = gpio.getState(); // either 0 or 1
    std::cout << state << std::endl;
    //    fflush(stdout);
    //    getchar();
}
