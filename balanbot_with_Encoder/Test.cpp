#include "Test.hpp"

int servo(int argc, char** argv) {
    ServoClass servo(4);

    //servo.setAngle(0);
    servo.setDutyCyclePercentage(3.5);
    servo.delay(500);
    servo.setAngle(90);
    servo.delay(500);
    //servo.setAngle(180);
    servo.setDutyCyclePercentage(15.5);
    servo.delay(500);

    for (int i = 0; i < 180; i++) {
        servo.setAngle(i);
        servo.delay(50);
    }
    return 0;
}

int led(int argc, char** argv) {
    LEDClass led(0);
    
    int i;
    for (i = 0; i <= 60; i++) {
        led.setDutyCyclePercentage(i);
        led.delay(50);
    }
    for (; i >= 0; i--) {
        led.setDutyCyclePercentage(i);
        led.delay(50);
    }
    sleep(2);

}

int motorDriver(int argc, char** argv) {
    MotorDriver motorA(0, 14, 15);
    MotorDriver motorB(1, 22, 23);
    GPIOClass stby(24);
    stby.outputMode();
    stby.setHigh();
    
    motorA.forward();
    motorB.forward();
    sleep(3);
    motorA.setDutyCyclePercentage(50);
    motorA.backward();
    motorB.setDutyCyclePercentage(50);
    motorB.backward();
    sleep(3);
    motorA.stop();
    motorB.stop();
}

inline long millis() {
    struct timeval millis;
    gettimeofday(&millis, NULL);
    return (millis.tv_usec + millis.tv_sec * 1000000) / 1000;
}

#define TIME_INTERVAL 2000

/*
long motorEncoder(int argc, char** argv) {
    unsigned long cur_time, prev_time;
    long prev_counter, counter;
    MotorEncoder motor(0, // pwm gpio
            14, //motor driver a1 gpio
            15, //motor driver a2 gpio
            20, //motor encoder input-a gpio,
            26 //motor encoder input-b gpio
            );
    
    GPIOClass stby(24);
    stby.outputMode();
    stby.setHigh();
    
    motor.forward();
    prev_time = millis();
    prev_counter = -99;
    do {
        counter = motor.readCounter();
        if (counter != prev_counter) std::cout << counter << "\r\n";
        prev_counter=counter;
    } while ((cur_time = millis()) < prev_time + TIME_INTERVAL);

    motor.stop();
}
 */