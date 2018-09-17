#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif
#include <unistd.h>
#ifdef __cplusplus
}
#endif
#include "ServoClass.hpp"


#define SERVO_MAX 2500000
#define SERVO_MIN 600000
#define PWM(angle) ((int) (SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180.0))

ServoClass::ServoClass() {
}

ServoClass::ServoClass(const ServoClass& orig) {
}

ServoClass::ServoClass(int gpio) : SoftPWM(gpio) {
    setFrequency(60);
}

void ServoClass::setAngle(int angle) {
    setDutyCycle(PWM(angle));
}

ServoClass::~ServoClass() {
}

