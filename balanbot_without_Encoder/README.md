# Introduction #
This is the Netbeans project that contains examples to use [libmraa](https://github.com/intel-iot-devkit/mraa) and [libupm](https://github.com/intel-iot-devkit/upm). 

The examples are run and compiled using NetBeans [C/C++ Remote Development](https://netbeans.org/kb/docs/cnd/remotedev-tutorial.html) feature for Raspberry Pi.

Using following linking option
```
-lmraa -lm -pthread -lupm-pca9685 -lupm-i2clcd -lupm-mpu9150 -lupm-nunchuck
```