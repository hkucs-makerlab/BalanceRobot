#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif
#include <unistd.h>
#include <sys/time.h>
#ifdef __cplusplus
}
#endif

#include "ServoClass.hpp"
#include "LEDClass.hpp"
#include "MotorEncoder.hpp"
#include "GPIOClass.hpp"
#include "MotorDriver.hpp"
using namespace std;

int servo(int argc, char** argv); 

int led(int argc, char** argv); 

int motorDriver(int argc, char** argv);

inline long millis();

#define TIME_INTERVAL 2000

long motorEncoder(int argc, char** argv); 