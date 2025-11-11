/**
********************************************************************************
 * @file motor_config.h
 * @author MTE380-Group
 * @date 2025-11
 * @brief Configuration header file for servos
 ********************************************************************************
 */

#pragma once

/************************************
 * INCLUDES
 ************************************/
#include <Adafruit_PWMServoDriver.h>

/************************************
 * MACROS AND DEFINES
 ************************************/
#define OSCILLATOR_FREQ 27000000
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define PULSELEN_COUNT 4096
#define SERVOMIN  350 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  250 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMID  300

#define NUM_SERVOS  3 
#define SERVO1      0
#define SERVO2      1
#define SERVO3      2

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/
