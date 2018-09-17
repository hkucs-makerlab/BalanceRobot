/* 
 * File:   main.cpp
 * Author: user
 *
 * Created on May 31, 2016, 3:16 PM
 */

#include <cstdlib>
#include <cstdio>
#include <cmath>
//#include <math>
#include <cstring>
#include <iostream>
#include <string>
#include <mraa.hpp>
#include <sys/time.h>
#include <pthread.h>
#include <stdarg.h> //print function

#include <upm/pca9685.hpp>
#include <upm/mpu9250.hpp>
#include <upm/nunchuck.hpp>
#include <upm/lcm1602.hpp>
#include "KalmanFilter.hpp"
#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "RotaryEncoder.hpp"
#include "PIDClass.hpp"
#include "Identity.hpp"
#include "UDPClass.hpp"

using namespace std;

#ifdef	__cplusplus
extern "C" {
#endif
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <getopt.h>
#ifdef	__cplusplus
}
#endif 

Identity id;
UDPClass udp;

PIDClass controller;
KalmanFilter kalmanFilter;
MotorEncoder encoder;

PID_t pitchPID[2];  //PID Controllers for pitch angle
float pitchPIDoutput[2] = {0};
float pidP_P_GAIN, pidP_I_GAIN, pidP_D_GAIN, pidP_I_LIMIT, pidP_EMA_SAMPLES;

PID_t speedPID[2];  //PID Controllers for wheel speed
float speedPIDoutput[2] = {0};
float pidS_P_GAIN, pidS_I_GAIN, pidS_D_GAIN, pidS_I_LIMIT, pidS_EMA_SAMPLES;

PID_t oriPID;   // PID Controller for orientation 
float oriOutput = 0;
float pidOri_P_GAIN(0.03), pidOri_I_GAIN(600), pidOri_D_GAIN(50), pidOri_I_LIMIT(500), pidOri_EMA_SAMPLES(2); //p= 0.03

#define VERSION 2 // 1 for red one, 2 for blue one
#define BLUETOOTH

#define PIDS_OUTPUT_LIMIT 20.0f     // max angle
#define SENSITIVITY 0.5f  // the range of PWM that keep its original value
#define STEP_LIMIT 300.0f   // max control step

#define PA 18   // PWM-A
#define A1 7    // Motor A
#define A2 25
#define PB 19   // PWM-B
#define B1 22   //Motor B
#define B2 23
#define STBY 24     // Standby
#define EA1 6   // Encoder A
#define EA2 5
#define EB1 20  // Encoder B
#define EB2 26

float motor_min = 0.0f;
float zero = 2.2f;
int counter = 0;
int counter2 = 0;
float driveTrim = 0;
float turnTrim = 0;
float maxDrive = 40;
float maxTurn = 10;
double smoothedDriveTrim = 0;
double smoothedTurnTrim = 0;

int running = 1;
int inRunAwayState = 0;
int inFalloverState = 0; //Used to flag when Eddie has fallen over and disables motors
int inSteadyState = 0; //Used to flag how long Eddie is being held upright in a steady state and will enable motors
int StreamData = 0;

int i2cbus_id = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float kalmanAngle;
upm::MPU9250 *imu = new upm::MPU9250(i2cbus_id);

#define TEST_ENCODER true
//#define DISABLE_MOTORS
/*
// pid controlor for angle
#define PIDP_P_GAIN 8.0f    //8.0
#define PIDP_I_GAIN 250.0f  //250
#define PIDP_D_GAIN -30.0f   //-30
float PIDP_I_LIMIT = 50.0; //Ilimit is before process gain  10PIDP_P_GAIN
 */

//pid controllor for speed
/*
#define PIDS_P_GAIN 0.02f    //0.02
#define PIDS_I_GAIN 0.0f  //800
#define PIDS_D_GAIN -160.0f  //-160
#define PIDS_EMA_SAMPLES 10.0f
#define PIDS_I_LIMIT  450.0
 */



enum {
    CONSOLE = 0,
    UDP //Will send all print() calls to UDP port 4243, all printf() will still go to console (:)
};
int outputto = UDP; //Change to fit current need.

// Define the exit signal handler




/* Incoming UDP Control Packet handler */
void UDP_Control_Handler(char * p_udpin) {
    char response[128] = {0};
    //cout << "Control:　"<< p_udpin << endl;
    if (memcmp(p_udpin, "DISCOVER", 8) == 0) {
        sprintf(response, "DISCOVER: %s", id.thisEddieName);
    } else if (strncmp(p_udpin, "SETNAME", 7) == 0) {
        id.setName(&p_udpin[7]);
        sprintf(response, "SETNAME: %s", id.thisEddieName);
    } else if (memcmp(p_udpin, "BIND", 4) == 0) {
        udp.setCommandBindAddress();
        sprintf(response, "BIND: OK");
    }

    udp.UDPCtrlSend(response);
}

/* Incoming UDP Command Packet handler:
 *
 * DRIVE[value]	=	+ is Forwards, - is Reverse, 0.0 is IDLE
 * TURN[value]	=	+ is Right, - is Left, 0.0 is STRAIGHT
 *
 * SETPIDS = Changes all PIDs for speed and pitch controllers
 * GETPIDS = Returns all PIDs for speed and pitch controllers via UDP
 *
 * PIDP[P,I,D][value] = adjust pitch PIDs
 * SPID[P,I,D][value] = adjust speed PIDs
 *
 * KALQA[value] = adjust Kalman Q Angle
 * KALQB[value] = adjust Kalman Q Bias
 * KALR[value] = adjust Kalman R Measure
 *
 * STOPUDP	= Will stop Eddie from sending UDP to current recipient
 *
 * STREAM[0,1] = Enable/Disable Live Data Stream
 *
 */



int print(const char *format, ...) {
    static char buffer[2048];

    va_list arg;
    int len;

    va_start(arg, format);
    len = vsprintf(buffer, format, arg);
    va_end(arg);

    if (len <= 0) return len;

    switch (outputto) {
        case CONSOLE:
            printf("%s", buffer);
            break;
        case UDP:
            udp.UDPBindSend(buffer, len);
            break;
    }

    return len;
}

void signal_callback_handler(int signum) {
    print("Exiting program; Caught signal %d\r\n", signum);
    running = false;
}

void UDP_Command_Handler(char * p_udpin) {
    //cout << "command:　"<< p_udpin << endl;
    //printf("the command is:%s  ", p_udpin);
    /* DRIVE commands */
    if (memcmp(p_udpin, "DRIVE", 5) == 0) {
        driveTrim = atof(&p_udpin[5]);
    }/* TURN commands */
    else if (memcmp(p_udpin, "TURN", 4) == 0) {
        turnTrim = atof(&p_udpin[4]);
    }/* Get/Set all PID quick commands*/
    else if (strncmp(p_udpin, "SETPIDS:", 8) == 0) {
        char * pch;

        pch = strtok(&p_udpin[8], ",");
        pidP_P_GAIN = atof(pch);

        pch = strtok(NULL, ",");
        pidP_I_GAIN = atof(pch);

        pch = strtok(NULL, ",");
        pidP_D_GAIN = atof(pch);

        pch = strtok(NULL, ",");
        pidS_P_GAIN = atof(pch);

        pch = strtok(NULL, ",");
        pidS_I_GAIN = atof(pch);

        pch = strtok(NULL, ",");
        pidS_D_GAIN = atof(pch);
    } else if (strncmp(p_udpin, "GETPIDS", 7) == 0) {
        print("CURRENTPIDS:%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", pidP_P_GAIN, pidP_I_GAIN, pidP_D_GAIN, pidS_P_GAIN, pidS_I_GAIN, pidS_D_GAIN);
    }/* Individual Pitch PID Controller commands */
    else if (strncmp(p_udpin, "PPIDP", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Pitch PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidP_P_GAIN, newGain);
        pidP_P_GAIN = newGain;
    } else if (strncmp(p_udpin, "PPIDI", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Pitch PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidP_I_GAIN, newGain);
        pidP_I_GAIN = newGain;
    } else if (strncmp(p_udpin, "PPIDD", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Pitch PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidP_D_GAIN, newGain);
        pidP_D_GAIN = newGain;
    }/* Individual Speed PID Controller commands*/
    else if (strncmp(p_udpin, "SPIDP", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Speed PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidS_P_GAIN, newGain);
        pidS_P_GAIN = newGain;
    } else if (strncmp(p_udpin, "SPIDI", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Speed PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidS_I_GAIN, newGain);
        pidS_I_GAIN = newGain;
    } else if (strncmp(p_udpin, "SPIDD", 5) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[5]);
        print("New Speed PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidS_D_GAIN, newGain);
        pidS_D_GAIN = newGain;
    }/* Pitch Kalman filter tuning commands */
    else if (strncmp(p_udpin, "KALQA", 4) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[4]);
        print("Setting Kalman Q Angle to: %0.4f\r\n", newGain);
        kalmanFilter.setQkalmanangle(newGain);
    } else if (strncmp(p_udpin, "KALQB", 4) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[4]);
        print("Setting Kalman Q Bias to: %0.4f\r\n", newGain);
        kalmanFilter.setQbias(newGain);
    } else if (strncmp(p_udpin, "KALR", 4) == 0) {
        float newGain = 0;
        newGain = atof(&p_udpin[4]);
        print("Setting Kalman R Measure to: %0.4f\r\n", newGain);
        kalmanFilter.setRmeasure(newGain);
    }/* UDP Hangup command */
    else if (strncmp(p_udpin, "STOPUDP", 7) == 0) {
        udp.UDPCloseTX();
    }/* Enable/Disable live data stream */
    else if (strncmp(p_udpin, "STREAM1", 7) == 0) {
        StreamData = 1;
    } else if (strncmp(p_udpin, "STREAM0", 7) == 0) {
        StreamData = 0;
    }
}

    
    
void getOrientation(float &pitch,float &roll) {
    imu->update();
    imu->getGyroscope(&gx, &gy, &gz);
    imu->getAccelerometer(&ax, &ay, &az);
    imu->getMagnetometer(&mx, &my, &mz);
    
    float i2cPitch, i2cRoll;
    float const PI_F = 3.14159265F;
    
    if(VERSION == 1)                 
        i2cRoll = (float) atan2(ay, az);
    else if(VERSION == 2)
        i2cRoll = (float) atan2(az, -ay);  // ay -> az, az -> -ay
    
    if (ay * sin(i2cRoll) + az * cos(i2cRoll) == 0)
        i2cPitch = ax > 0 ? (PI_F / 2) : (-PI_F / 2);
    else
        i2cPitch = (float) atan(-ax / (ay * sin(i2cRoll) + az * cos(i2cRoll)));
                                
    float i2cHeading = (float) atan2(mz * sin(i2cRoll) - my * cos(i2cRoll), mx * cos(i2cPitch) + my * sin(i2cPitch) * sin(i2cRoll) + mz * sin(i2cPitch) * cos(i2cRoll));

    // Convert angular data to degree 
    i2cRoll = -i2cRoll * 180.0 / PI_F;
    i2cPitch = i2cPitch * 180.0 / PI_F;
    i2cHeading = -i2cHeading * 180.0 / PI_F;
    roll = i2cRoll;
    pitch = i2cPitch;
}


double current_milliseconds() {
    struct timeval c_time;
    gettimeofday(&c_time, NULL);
    double milliseconds = c_time.tv_sec * 1000 + c_time.tv_usec / 1000;
    return milliseconds;
}

void* readEncoder(void* prt){
    encoder.initEncoders(6,5,20,26);
    double* temp;
    temp = new double[2];
    while(true){     
        encoder.GetEncoders(temp);       
        //counter = encoder.GetEncoder();
        counter = temp[0];
        counter2 = temp[1];
        //std::cout << "encoderA: " << temp[0] <<"   encoderB: "<<temp[1]<< "\r\n";               
        usleep(10);
    }
}
void* runBlueTooth(void* prt){
    bool setup = true;
    string uart_port = "/dev/ttyUSB0";
    mraa_uart_context uart;
    uart = mraa_uart_init_raw(uart_port.c_str());
    mraa_uart_set_baudrate(uart, 115200);
    mraa_uart_set_mode(uart, 32, MRAA_UART_PARITY_NONE, 1);

    if (uart == NULL) {
        fprintf(stderr, "UART failed to setup %s\n", uart_port.c_str());
        setup = false;
        //return EXIT_FAILURE;
    }

    const int delay_ms = 10;
    while(setup){
        char buffer[18];
            //sprintf(buffer, "#0 P%d T%d\r\n", PWM(angle), delay_ms);
            //std::cout << i << std::endl;
        mraa_uart_read(uart, buffer, 18);
        //std::cout << buffer << std::endl << "\r";
        char* mode;
        mode = strtok(buffer," ;,\r\n\0");
        double x,y;
        if(mode != NULL){
            if(strncmp(mode, "CM", 2) == 0){
                x = -atof(strtok(NULL," ;,"));
                y = atof(strtok(NULL," ;,"));
                if(x > 45) x = 45;
                else if(x < -45) x = -45;           
                if(y > 45) y = 45;
                else if(y < -45) y = -45;

                driveTrim = maxDrive * x / 45;
                turnTrim = -maxTurn * y / 45;
            }else if(strncmp(mode, "CJ", 2) == 0){
                x = -atof(strtok(NULL," ;,"));
                y = -atof(strtok(NULL," ;,"));
                driveTrim = y * maxDrive;
                turnTrim = x * maxTurn;
            }else if(strncmp(mode, "CS", 2) == 0){
                driveTrim = turnTrim = 0;
            }
        }
        usleep(delay_ms * 1000);       
    }
    mraa_uart_stop(uart);

    //return 0;
}
int map(int input, int min_1, int max_1, int min_2, int max_2){
    int output;
    if(input > 0)
        output = min_2+(max_2 - min_2)*input/(max_1 - min_1);
    else{
        output = min_2+(max_2 - min_2)*(-input)/(max_1 - min_1);
        output = -output;
    }
    return output;
}

int main(int argc, char** argv) {
    
    //Register signal and signal handler
    signal(SIGINT, signal_callback_handler);
    signal(SIGHUP, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);

    //Init UDP with callbacks and pointer to run status
    udp.initUDP(&UDP_Command_Handler, &UDP_Control_Handler);
    
    print("Eddie starting...\r\n");
    id.initIdentity();
    
    
    
    // building thread to read encoder
    pthread_t encoderThread, udplistenerThread;
#ifdef BLUETOOTH
    pthread_t bluetoothThread;
    char* mg;
    int iret_b = pthread_create(&bluetoothThread, NULL, runBlueTooth , (void*)mg);
#endif
    char* msg;
    int iret = pthread_create(&encoderThread, NULL, readEncoder, (void*)msg);
    if(iret){
        fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
        exit(EXIT_FAILURE);
    }
    print("Encoders activated.\r\n");
    
    pthread_create(&udplistenerThread, NULL, udplistenerThreadHandler, &udp);   //can't create pthread inside function of udp
    print("Eddie is starting the UDP network thread..\r\n");

    mraa_init();  
#ifndef DISABLE_MOTORS
    print("Starting motor driver (and resetting wireless) please be patient..\r\n");
    MotorDriver motorA(PA,A1,A2);
    MotorDriver motorB(PB,B1,B2);   
    GPIOClass stby(STBY);
    stby.outputMode();
    stby.setHigh();
    print("Motor Driver Started.\r\n");
#endif
    
    
    print("Eddie is Starting PID controllers\r\n");
    pidP_P_GAIN = PIDP_P_GAIN;   
    pidP_I_GAIN = PIDP_I_GAIN;  
    pidP_D_GAIN = PIDP_D_GAIN;  
    pidP_I_LIMIT = PIDP_I_LIMIT;
    pidP_EMA_SAMPLES = PIDP_EMA_SAMPLES;
    controller.PIDinit(&pitchPID[0], &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES);
    controller.PIDinit(&pitchPID[1], &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES);
    
    pidS_P_GAIN = PIDS_P_GAIN;   
    pidS_I_GAIN = PIDS_I_GAIN;  
    pidS_D_GAIN = PIDS_D_GAIN;  
    pidS_I_LIMIT = PIDS_I_LIMIT;
    pidS_EMA_SAMPLES = PIDS_EMA_SAMPLES;
    controller.PIDinit(&speedPID[0], &pidS_P_GAIN, &pidS_I_GAIN, &pidS_D_GAIN, &pidS_I_LIMIT, &pidS_EMA_SAMPLES);
    controller.PIDinit(&speedPID[1], &pidS_P_GAIN, &pidS_I_GAIN, &pidS_D_GAIN, &pidS_I_LIMIT, &pidS_EMA_SAMPLES);
    
    float PID_S_output = 0;
 
    controller.PIDinit(&oriPID,&pidOri_P_GAIN, &pidOri_I_GAIN, &pidOri_D_GAIN, &pidOri_I_LIMIT, &pidOri_EMA_SAMPLES);
   
    double gy_scale = 0.01;
    double last_PID_ms, last_gy_ms;
    float filteredPitch, filteredRoll;
    
    imu->init();
    print("IMU Started.\r\n");
    
    getOrientation(filteredPitch, filteredRoll);
    kalmanFilter.InitKalman();
    float kalmanAngle;
    if(VERSION == 1){
        kalmanAngle = filteredPitch;
        kalmanFilter.setkalmanangle(filteredPitch);
    }else{
        kalmanAngle = filteredRoll;
        kalmanFilter.setkalmanangle(filteredRoll);
    }
    
    
    float PID_output = 0;
    last_PID_ms = last_gy_ms = current_milliseconds();
    
    
    
    
    print("Eddie startup complete. Hold me upright to begin\r\n");
    float lastTurnTrim = smoothedTurnTrim;
    while (running) {
        if (fabs(encoder.GetEncoder()) > 3000 && !inRunAwayState) {
            print("Help! I'm running and not moving.\r\n");
            encoder.ResetEncoders();
            inRunAwayState = 1;
        }
 
        gy_scale = (current_milliseconds() - last_gy_ms)/1000.0f;
        last_gy_ms = current_milliseconds();
        float newPitch, newRoll;
        getOrientation(newPitch,newRoll);
        imu->getGyroscope(&gx, &gy, &gz);   // used to get gy
        if(VERSION == 1){
            filteredPitch = 0.995 * (filteredPitch + (gy*gy_scale)) + (0.005 * newPitch); //0.98 & 0.02        
            kalmanAngle = kalmanFilter.getkalmanangle(filteredPitch, gy, gy_scale); // lack of negative sign
            
        }else{
            gx = -gx;
            filteredRoll = 0.995 * (filteredRoll + (gx*gy_scale)) + (0.005 * newRoll); //0.98 & 0.02        
            kalmanAngle = kalmanFilter.getkalmanangle(filteredRoll, gx, gy_scale); // lack of negative sign
            
            
        }
        //std::cout << "encoderA: " << counter <<"   encoderB: "<<counter2<< "\r\n";
        //std::cout << "Kalman_Angle:   " << kalmanAngle - zero << endl << "\r";
        //std::cout << "heading:   " << kalmanAngle << endl<<"\r";
        if ((inRunAwayState || (fabs(kalmanAngle) > 50)) && !inFalloverState) {
            #ifndef DISABLE_MOTORS
            stby.setLow();
            #endif
            inFalloverState = 1;
            print("Help! I've fallen over and I can't get up =)\r\n");
        } else if (fabs(kalmanAngle) < 10 && inFalloverState) {
            if (++inSteadyState == 100) {
                inRunAwayState = 0;
                inSteadyState = 0;
                #ifndef DISABLE_MOTORS
                stby.setHigh();
                #endif
                inFalloverState = 0;
                print("Thank you!\r\n");
            }
        } else {
            inSteadyState = 0;
        }
        if(!inFalloverState){
            //Use PID to calculate the target angle from the difference btw position        
            //calculate S_P
            double timenow = current_milliseconds();       
            double during = timenow - last_PID_ms;
            /* Drive operations */
            smoothedDriveTrim = (0.97 * smoothedDriveTrim) + (0.03 * driveTrim);
            //smoothedDriveTrim = driveTrim;
            if (smoothedDriveTrim != 0) {
                encoder.EncoderAddPos(smoothedDriveTrim); //Alter encoder position to generate movement
            }
            /* Turn operations */
            smoothedTurnTrim = turnTrim;
            if(smoothedTurnTrim != 0){
                encoder.EncoderAddPos2(-smoothedTurnTrim, smoothedTurnTrim);
            }
            /* Calculate pid output */
            speedPIDoutput[0] = controller.PIDUpdate(0, counter, timenow - last_PID_ms, &speedPID[0]);
            speedPIDoutput[1] = controller.PIDUpdate(0, counter2, timenow - last_PID_ms, &speedPID[1]);
            // Constraint angle of pitch 
            /*
            if(PID_S_output > PIDS_OUTPUT_LIMIT)
                PID_S_output = PIDS_OUTPUT_LIMIT;
            else if(PID_S_output < -PIDS_OUTPUT_LIMIT)
                PID_S_output = -PIDS_OUTPUT_LIMIT;
             */
            int posDiff = counter - counter2;
            oriOutput = controller.PIDUpdate(0, posDiff, timenow - last_PID_ms, &oriPID);
            pitchPIDoutput[0] = controller.PIDUpdate(speedPIDoutput[0] - oriOutput, kalmanAngle - zero, timenow - last_PID_ms, &pitchPID[0]);
            pitchPIDoutput[1] = controller.PIDUpdate(speedPIDoutput[1] + oriOutput, kalmanAngle - zero, timenow - last_PID_ms, &pitchPID[1]);
            
            

            last_PID_ms = timenow;       
            // ensure pwm is a valid value 
            if (pitchPIDoutput[0] > 100.0) pitchPIDoutput[0] = 100.0;
            if (pitchPIDoutput[1] > 100.0) pitchPIDoutput[1] = 100.0;
            if (pitchPIDoutput[0] < -100.0) pitchPIDoutput[0] = -100.0;
            if (pitchPIDoutput[1] < -100.0) pitchPIDoutput[1] = -100.0;
            /*
            // set the minumum pwm to ensure motor can rotate        
            if(pitchPIDoutput[0] < motor_min && pitchPIDoutput[0] > SENSITIVITY) pitchPIDoutput[0] = motor_min;
            else if(pitchPIDoutput[0] > -motor_min && pitchPIDoutput[0] < -SENSITIVITY) pitchPIDoutput[0] = - motor_min;
            if(pitchPIDoutput[1] < motor_min && pitchPIDoutput[1] > SENSITIVITY) pitchPIDoutput[1] = motor_min;
            else if(pitchPIDoutput[1] > -motor_min && pitchPIDoutput[1] < -SENSITIVITY) pitchPIDoutput[1] = - motor_min;
            */
            // map the value of pwm(0-100) to (20-100)
            pitchPIDoutput[0] = map(pitchPIDoutput[0], 0, 100, motor_min, 100);
            pitchPIDoutput[1] = map(pitchPIDoutput[1], 0, 100, motor_min, 100);

            
        }else{  //We are inFalloverState
            encoder.ResetEncoders();
            pitchPID[0].accumulatedError = 0;
            pitchPID[1].accumulatedError = 0;
            speedPID[0].accumulatedError = 0;
            speedPID[1].accumulatedError = 0;
            driveTrim = 0;
            turnTrim = 0;
        }
        #ifndef DISABLE_MOTORS
        /* feedback to the motor */
        if(pitchPIDoutput[0] < 0){
            motorA.setDutyCyclePercentage(abs(pitchPIDoutput[0]));
            motorA.backward();
        }else{           
            motorA.setDutyCyclePercentage(pitchPIDoutput[0]);     
            motorA.forward();  
        }
        if(pitchPIDoutput[1] < 0){
            motorB.setDutyCyclePercentage(abs(pitchPIDoutput[1]));
            motorB.backward();
        }else{           
            motorB.setDutyCyclePercentage(pitchPIDoutput[1]);     
            motorB.forward();  
        }       
        #endif

        if ((!inFalloverState || outputto == UDP) && StreamData) {
            print("PIDout: %0.2f,%0.2f\tcompPitch: %6.2f kalPitch: %6.2f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\r\n",
                    speedPIDoutput[0],
                    pitchPIDoutput[0],
                    filteredPitch,
                    kalmanAngle,
                    pitchPID[0].error,
                    pitchPID[0].accumulatedError,
                    pitchPID[0].differentialError,
                    speedPID[0].error,
                    speedPID[0].accumulatedError,
                    speedPID[0].differentialError
                    );
        }
        //usleep(1000);//at most 50
    }
    
    delete imu;   
    mraa_deinit();
    print("Eddie is cleaning up...\r\n");
    udp.stop();
    encoder.CloseEncoder();
#ifndef DISABLE_MOTORS
    stby.setLow();
    print("Motor Driver Disabled..\r\n");
#endif
    print("Eddie cleanup complete. Good Bye!\r\n");
    std::cout << "ended!\n" << std::endl;
    return 0;
   
}
