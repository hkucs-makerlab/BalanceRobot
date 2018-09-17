#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/GPIOClass.o \
	${OBJECTDIR}/Identity.o \
	${OBJECTDIR}/KalmanFilter.o \
	${OBJECTDIR}/LEDClass.o \
	${OBJECTDIR}/MotorDriver.o \
	${OBJECTDIR}/MotorEncoder.o \
	${OBJECTDIR}/PIDClass.o \
	${OBJECTDIR}/RotaryEncoder.o \
	${OBJECTDIR}/ServoClass.o \
	${OBJECTDIR}/SoftPWM.o \
	${OBJECTDIR}/Test.o \
	${OBJECTDIR}/UDPClass.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L/usr/local/lib -lmraa -lm -pthread -lupm-pca9685 -lupm-i2clcd -lupm-mpu9150 -lupm-nunchuck -lupm-lcm1602

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/balanbot_without_encoder

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/balanbot_without_encoder: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	g++ -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/balanbot_without_encoder ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/GPIOClass.o: GPIOClass.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/GPIOClass.o GPIOClass.cpp

${OBJECTDIR}/Identity.o: Identity.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Identity.o Identity.cpp

${OBJECTDIR}/KalmanFilter.o: KalmanFilter.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/KalmanFilter.o KalmanFilter.cpp

${OBJECTDIR}/LEDClass.o: LEDClass.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LEDClass.o LEDClass.cpp

${OBJECTDIR}/MotorDriver.o: MotorDriver.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/MotorDriver.o MotorDriver.cpp

${OBJECTDIR}/MotorEncoder.o: MotorEncoder.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/MotorEncoder.o MotorEncoder.cpp

${OBJECTDIR}/PIDClass.o: PIDClass.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PIDClass.o PIDClass.cpp

${OBJECTDIR}/RotaryEncoder.o: RotaryEncoder.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/RotaryEncoder.o RotaryEncoder.cpp

${OBJECTDIR}/ServoClass.o: ServoClass.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ServoClass.o ServoClass.cpp

${OBJECTDIR}/SoftPWM.o: SoftPWM.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SoftPWM.o SoftPWM.cpp

${OBJECTDIR}/Test.o: Test.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Test.o Test.cpp

${OBJECTDIR}/UDPClass.o: UDPClass.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/UDPClass.o UDPClass.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include/mraa -I/usr/local/include/upm -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
