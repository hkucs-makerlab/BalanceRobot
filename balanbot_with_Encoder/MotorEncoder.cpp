/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MotorEncoder.cpp
 * Author: user
 * 
 * Created on January 30, 2017, 2:17 PM
 */

#include "MotorEncoder.hpp"

void EncoderInterruptA(void * args) {
    MotorEncoder *encoder = (MotorEncoder*) args;
    encoder->EncoderInterruptAA(args);
}

void EncoderInterruptB(void * args) {
    MotorEncoder *encoder = (MotorEncoder*) args;
    encoder->EncoderInterruptBB(args);
}

MotorEncoder::MotorEncoder() {
}

MotorEncoder::MotorEncoder(const MotorEncoder& orig) {
}

void MotorEncoder::initEncoders(int a, int b, int c, int d) {
    mraa_init();
    position[0] = position[1] = 0;
    encoderx[ 0 ] = mraa_gpio_init_raw(a);
    encoderx[ 1 ] = mraa_gpio_init_raw(b);
    encoderx[ 2 ] = mraa_gpio_init_raw(c);
    encoderx[ 3 ] = mraa_gpio_init_raw(d);

    mraa_gpio_dir(encoderx[ 0 ], MRAA_GPIO_IN);    // set direction of this pin context as in
    mraa_gpio_isr(encoderx[ 0 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptA, this);    // set an interrupt to this pin and set edge mode as both, set the trigger fuction and its argument
    mraa_gpio_dir(encoderx[ 1 ], MRAA_GPIO_IN);
    mraa_gpio_isr(encoderx[ 1 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptA, this);
    mraa_gpio_dir(encoderx[ 2 ], MRAA_GPIO_IN);
    mraa_gpio_isr(encoderx[ 2 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptB, this);
    mraa_gpio_dir(encoderx[ 3 ], MRAA_GPIO_IN);
    mraa_gpio_isr(encoderx[ 3 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptB, this);

}

void MotorEncoder::EncoderInterruptAA(void * args) {    // regard interrupt as encoder change
    int currentpins[ 2 ];    // current state of the pin

    int change = 0;
    currentpins[ 0 ] = mraa_gpio_read(encoderx[ 0 ]);    // read the value of the pin
    currentpins[ 1 ] = mraa_gpio_read(encoderx[ 1 ]);

    if (currentpins[ 0 ] != lastpins[ 0 ]) { // the first pin value has been changed
        if (currentpins[ 0 ] > lastpins[ 0 ]) {    // 0 > -1 or 1 > 0
            if (currentpins[ 1 ]) {    // consider second pin if it is 1 or -1
                --change;
            } else {    
                ++change;
            }
        } else {    // 0 < 1 or -1 < 0
            if (currentpins[ 1 ]) { //change in reverse direction in respect to above
                ++change;    
            } else {    
                --change;
            }
        }
    } else if (currentpins[ 1 ] != lastpins[ 1 ]) { // the first pin is unchanged, check second pin, and is changed
        if (currentpins[ 1 ] > lastpins[ 1 ]) {
            if (currentpins[ 0 ]) {    
                ++change;
            } else {
                --change;
            }
        } else {
            if (currentpins[ 0 ]) {
                --change;
            } else {
                ++change;
            }
        }
    }

    position[ 0 ] += change;

    lastpins[ 0 ] = currentpins[ 0 ];
    lastpins[ 1 ] = currentpins[ 1 ];
}

void MotorEncoder::EncoderInterruptBB(void * args) {
    int currentpins[ 2 ];

    int change = 0;
    currentpins[ 0 ] = mraa_gpio_read(encoderx[ 2 ]);
    currentpins[ 1 ] = mraa_gpio_read(encoderx[ 3 ]);

    if (currentpins[ 0 ] != lastpins[ 2 ]) {
        if (currentpins[ 0 ] > lastpins[ 2 ]) {
            if (currentpins[ 1 ]) {
                ++change;
            } else {
                --change;
            }
        } else {
            if (currentpins[ 1 ]) {
                --change;
            } else {
                ++change;
            }
        }
    } else if (currentpins[ 1 ] != lastpins[ 3 ]) {
        if (currentpins[ 1 ] > lastpins[ 3 ]) {
            if (currentpins[ 0 ]) {
                --change;
            } else {
                ++change;
            }
        } else {
            if (currentpins[ 0 ]) {
                ++change;
            } else {
                --change;
            }
        }
    }

    position[ 1 ] += change;
    lastpins[ 2 ] = currentpins[ 0 ];
    lastpins[ 3 ] = currentpins[ 1 ];
}

void MotorEncoder::ResetEncoders() {
    position[ 0 ] = position[ 1 ] = 0;
}

double MotorEncoder::GetEncoder() {
    return (position[ 0 ] + position[ 1 ]) / 2;
}

void MotorEncoder::GetEncoders(double * temp) {
    temp[0] = position[ 0 ];
    temp[1] = position[ 1 ];
}

void MotorEncoder::GetEncoderChange(double * temp) {
    temp[0] = position[ 0 ];
    temp[1] = position[ 1 ];
    position[ 0 ] = position[ 1 ] = 0;
}

void MotorEncoder::EncoderAddPos2(double distance1, double distance2) {
    position[0] += distance1;
    position[1] += distance2;
}

void MotorEncoder::EncoderAddPos(double distance) {
    position[0] += distance;
    position[1] += distance;
}

void MotorEncoder::CloseEncoder() {
    mraa_gpio_close(encoderx[ 0 ]);
    mraa_gpio_close(encoderx[ 1 ]);
    mraa_gpio_close(encoderx[ 2 ]);
    mraa_gpio_close(encoderx[ 3 ]);
}

MotorEncoder::~MotorEncoder() {
}

