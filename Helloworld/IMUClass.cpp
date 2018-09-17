/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   IMUClass.cpp
 * Author: user
 * 
 * Created on January 30, 2017, 5:34 PM
 */

#include "IMUClass.hpp"

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G   0x0F
#define CTRL_REG1_G   0x20
#define CTRL_REG2_G   0x21
#define CTRL_REG3_G   0x22
#define CTRL_REG4_G   0x23
#define CTRL_REG5_G   0x24
#define REFERENCE_G   0x25
#define STATUS_REG_G  0x27
#define OUT_X_L_G   0x28
#define OUT_X_H_G   0x29
#define OUT_Y_L_G   0x2A
#define OUT_Y_H_G   0x2B
#define OUT_Z_L_G   0x2C
#define OUT_Z_H_G   0x2D
#define FIFO_CTRL_REG_G  0x2E
#define FIFO_SRC_REG_G  0x2F
#define INT1_CFG_G   0x30
#define INT1_SRC_G   0x31
#define INT1_THS_XH_G  0x32
#define INT1_THS_XL_G  0x33
#define INT1_THS_YH_G  0x34
#define INT1_THS_YL_G  0x35
#define INT1_THS_ZH_G  0x36
#define INT1_THS_ZL_G  0x37
#define INT1_DURATION_G  0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM  0x05
#define OUT_TEMP_H_XM  0x06
#define STATUS_REG_M  0x07
#define OUT_X_L_M   0x08
#define OUT_X_H_M   0x09
#define OUT_Y_L_M   0x0A
#define OUT_Y_H_M   0x0B
#define OUT_Z_L_M   0x0C
#define OUT_Z_H_M   0x0D
#define WHO_AM_I_XM   0x0F
#define INT_CTRL_REG_M  0x12
#define INT_SRC_REG_M  0x13
#define INT_THS_L_M   0x14
#define INT_THS_H_M   0x15
#define OFFSET_X_L_M  0x16
#define OFFSET_X_H_M  0x17
#define OFFSET_Y_L_M  0x18
#define OFFSET_Y_H_M  0x19
#define OFFSET_Z_L_M  0x1A
#define OFFSET_Z_H_M  0x1B
#define REFERENCE_X   0x1C
#define REFERENCE_Y   0x1D
#define REFERENCE_Z   0x1E
#define CTRL_REG0_XM  0x1F
#define CTRL_REG1_XM  0x20
#define CTRL_REG2_XM  0x21
#define CTRL_REG3_XM  0x22
#define CTRL_REG4_XM  0x23
#define CTRL_REG5_XM  0x24
#define CTRL_REG6_XM  0x25
#define CTRL_REG7_XM  0x26
#define STATUS_REG_A  0x27
#define OUT_X_L_A   0x28
#define OUT_X_H_A   0x29
#define OUT_Y_L_A   0x2A
#define OUT_Y_H_A   0x2B
#define OUT_Z_L_A   0x2C
#define OUT_Z_H_A   0x2D
#define FIFO_CTRL_REG  0x2E
#define FIFO_SRC_REG  0x2F
#define INT_GEN_1_REG  0x30
#define INT_GEN_1_SRC  0x31
#define INT_GEN_1_THS  0x32
#define INT_GEN_1_DURATION 0x33
#define INT_GEN_2_REG  0x34
#define INT_GEN_2_SRC  0x35
#define INT_GEN_2_THS  0x36
#define INT_GEN_2_DURATION 0x37
#define CLICK_CFG   0x38
#define CLICK_SRC   0x39
#define CLICK_THS   0x3A
#define TIME_LIMIT   0x3B
#define TIME_LATENCY  0x3C
#define TIME_WINDOW   0x3D
#define ACT_THS    0x3E
#define ACT_DUR    0x3F


#define GYRO_I2C_ADDR 0x6B
#define XM_I2C_ADDR 0x1D

IMUClass::IMUClass() {
}

IMUClass::IMUClass(const IMUClass& orig) {
}

void IMUClass::sendi2c(unsigned int address, unsigned int reg, unsigned char tosend) {
    mraa_i2c_address(i2c, address);
    rx_tx_buf[0] = reg;
    rx_tx_buf[1] = tosend;
    mraa_i2c_write(i2c, rx_tx_buf, 2);
}

char IMUClass::readi2c(int address, int reg, int count) {
    int i = 0;
    mraa_i2c_address(i2c, address);
    for (i = 0; i < count; i++) {
        rx_tx_buf[i] = mraa_i2c_read_byte_data(i2c, reg + i);
    }
    if (count == 1)return rx_tx_buf[0];
    return 0;
}

void IMUClass::readGyro() {
    readi2c(GYRO_I2C_ADDR, OUT_X_L_G, 6); // Read 6 bytes, beginning at OUT_X_L_G
    float tgx = (float) ((short) (rx_tx_buf[1] << 8) | rx_tx_buf[0]) * (500.0 /*dps*/ / 32768.0);
    float tgy = (float) ((short) (rx_tx_buf[3] << 8) | rx_tx_buf[2]) * (500.0 /*dps*/ / 32768.0);
    float tgz = (float) ((short) (rx_tx_buf[5] << 8) | rx_tx_buf[4]) * (500.0 /*dps*/ / 32768.0);

    gx = tgz;
    gy = tgy;
    gz = tgx;
}

void IMUClass::readAccel() {
    readi2c(XM_I2C_ADDR, OUT_X_L_A, 6); // Read 6 bytes, beginning at OUT_X_L_G
    float tax = (float) ((short) (rx_tx_buf[1] << 8) | rx_tx_buf[0]) * 0.00006103515625; // Store x-axis values into gx
    float tay = (float) ((short) (rx_tx_buf[3] << 8) | rx_tx_buf[2]) * 0.00006103515625; // Store y-axis values into gy
    float taz = (float) ((short) (rx_tx_buf[5] << 8) | rx_tx_buf[4]) * 0.00006103515625; // Store z-axis values into gz

    ax = taz;
    ay = tay;
    az = tax;

    readi2c(XM_I2C_ADDR, OUT_TEMP_L_XM, 2);
    temp = (float) ((short) (rx_tx_buf[1] << 8) | rx_tx_buf[0]);
}

void IMUClass::readMag() {
    readi2c(XM_I2C_ADDR, OUT_X_L_M, 6); // Read 6 bytes, beginning at OUT_X_L_G
    float tmx = (float) ((short) (rx_tx_buf[1] << 8) | rx_tx_buf[0]) * 0.00006103515625; // Store x-axis values into gx
    float tmy = (float) ((short) (rx_tx_buf[3] << 8) | rx_tx_buf[2]) * 0.00006103515625; // Store y-axis values into gy
    float tmz = (float) ((short) (rx_tx_buf[5] << 8) | rx_tx_buf[4]) * 0.00006103515625; // Store z-axis values into gz

    mx = tmz;
    my = tmy;
    mz = tmx;
}

void IMUClass::readSensors() {
    readGyro();
    readAccel();
    readMag();
}

void IMUClass::getOrientation() {
    readSensors();

    float const PI_F = 3.14159265F;

    // i2cRoll: Rotation around the X-axis. -180 <= i2cRoll <= 180
    // a positive i2cRoll angle is defined to be a clockwise rotation about the positive X-axis
    //                    y
    //      i2cRoll = atan2(---)
    //                    z
    // where:  y, z are returned value from accelerometer sensor
    i2cRoll = (float) atan2(ay, az);

    // i2cPitch: Rotation around the Y-axis. -180 <= i2cRoll <= 180
    // a positive i2cPitch angle is defined to be a clockwise rotation about the positive Y-axis
    //                                 -x
    //      i2cPitch = atan(-------------------------------)
    //                    y * sin(i2cRoll) + z * cos(i2cRoll)
    // where:  x, y, z are returned value from accelerometer sensor
    if (ay * sin(i2cRoll) + az * cos(i2cRoll) == 0) i2cPitch = ax > 0 ? (PI_F / 2) : (-PI_F / 2);
    else i2cPitch = (float) atan(-ax / (ay * sin(i2cRoll) + az * cos(i2cRoll)));

    // i2cHeading: Rotation around the Z-axis. -180 <= i2cRoll <= 180
    // a positive i2cHeading angle is defined to be a clockwise rotation about the positive Z-axis
    //                                       z * sin(i2cRoll) - y * cos(i2cRoll)
    //   i2cHeading = atan2(--------------------------------------------------------------------------)
    //                    x * cos(i2cPitch) + y * sin(i2cPitch) * sin(i2cRoll) + z * sin(i2cPitch) * cos(i2cRoll))
    // where:  x, y, z are returned value from magnetometer sensor
    //  i2cHeading = (float)atan2(mz * sin(i2cRoll) - my * cos(i2cRoll), mx * cos(i2cPitch) + my * sin(i2cPitch) * sin(i2cRoll) + mz * sin(i2cPitch) * cos(i2cRoll));

    // Convert angular data to degree
    i2cRoll = -i2cRoll * 180.0 / PI_F;
    i2cPitch = i2cPitch * 180.0 / PI_F;
    i2cHeading = -i2cHeading * 180.0 / PI_F;

}

void IMUClass::imuinit() {
    mraa_init();
    i2c = mraa_i2c_init(1);

    sendi2c(GYRO_I2C_ADDR, FIFO_CTRL_REG_G, 0);
    sendi2c(GYRO_I2C_ADDR, CTRL_REG1_G, 0x0F); //Normal mode, enable all axes //0xFF ); //??unknown config??
    sendi2c(GYRO_I2C_ADDR, CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
    sendi2c(GYRO_I2C_ADDR, CTRL_REG4_G, 0x10); // Set scale to 500 dps
    sendi2c(GYRO_I2C_ADDR, CTRL_REG5_G, 0x00); // FIFO Disabled, HPF Disabled

    sendi2c(XM_I2C_ADDR, FIFO_CTRL_REG, 0);
    sendi2c(XM_I2C_ADDR, CTRL_REG1_XM, 0xFF);
    sendi2c(XM_I2C_ADDR, CTRL_REG2_XM, 0x00); //Set scale +/-2g
    sendi2c(XM_I2C_ADDR, CTRL_REG4_XM, 0x30);

    sendi2c(XM_I2C_ADDR, CTRL_REG5_XM, 0x94);
    sendi2c(XM_I2C_ADDR, CTRL_REG6_XM, 0x00);
    sendi2c(XM_I2C_ADDR, CTRL_REG7_XM, 0x00);
    /*
            return;

      while(1)
            {
                    readGyro();
                    readAccel();
                    readMag();

                    printf("gx:%6.2f gy:%6.2f gz:%6.2f  ax:%6.2f ay:%6.2f az:%6.2f  mx:%6.2f my:%6.2f mz:%6.2f  temp:%0.0f\n",gx,gy,gz,ax,ay,az,mx,my,mz,temp);
                    usleep(20000);
            }
     */
}

IMUClass::~IMUClass() {
}

