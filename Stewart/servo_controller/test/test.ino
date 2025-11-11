/**
 ********************************************************************************
 * @file test.ino
 * @author MTE380-Group  
 * @brief  Test file emulating tx transmissions to check message parsing
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
// Arduino
#include <Wire.h>

// Project
#include "motor_config.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
#define MSG_START_BYTE 0xAA
#define MSG_LENGTH 5
#define SERIAL_BAUDRATE 115200

/************************************
 * TYPEDEFS
 ************************************/
// Message struct
typedef struct
{
  uint8_t start;
  uint8_t data[MSG_LENGTH - 1];
} msg_t;

/************************************
 * Static Variables
 ************************************/
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/************************************
 * Static Functions
 ************************************/
static uint8_t checksum(const uint8_t* data, const uint8_t checksum_val);
static inline uint16_t clampPulse(uint16_t p);
static msg_t genMsg();

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void setup() 
{
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Starting Servo Controller Setup");

  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_FREQ);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.setPWM(SERVO1, 0, SERVOMID);
  pwm.setPWM(SERVO2, 0, SERVOMID);
  pwm.setPWM(SERVO3, 0, SERVOMID);

  delay(10);
}

//-----------------------------------------------------------------------
void loop() 
{
  // Generator Artificial Message
  msg_t msg = genMsg(); // Will print the generated messages
  
  // If new motor command is sent, process next three bytes
  if (msg.start == MSG_START_BYTE)
  {
    int bytes_read = 4;

    // Validate message
    if ( (bytes_read == MSG_LENGTH - 1) && (checksum(msg.data, msg.data[3])) )
    {
      // Interpret as signed angles (two’s complement)
      int8_t raw_a1 = (int8_t)msg.data[0];
      int8_t raw_a2 = (int8_t)msg.data[1];
      int8_t raw_a3 = (int8_t)msg.data[2];

      // Pulse = SERVO_MID + (-2.5)*angle (integer math, no float)
      uint16_t m1_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a1) / 10);
      uint16_t m2_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a2) / 10);
      uint16_t m3_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a3) / 10);

      #if SEND_ANGLE_PWM 
        pwm.setPWM(0, 0, m1_pulse);
        pwm.setPWM(1, 0, m2_pulse);
        pwm.setPWM(2, 0, m3_pulse);
      #endif

      // Print debug info over Serial
      Serial.print("Angle 1: "); 
      Serial.print(raw_a1);
      Serial.print(" | Pulse: ");
      Serial.println(m1_pulse);
      
      Serial.print("Angle 2: "); 
      Serial.print(raw_a2);
      Serial.print(" | Pulse: ");
      Serial.println(m2_pulse);
      
      Serial.print("Angle 3: "); 
      Serial.print(raw_a3);
      Serial.print(" | Pulse: ");
      Serial.println(m3_pulse);

      Serial.print("Checksum Valid: ");
      Serial.println(checksum(msg.data, msg.data[3]));
      Serial.println("----------------------------------");
    }
  }
  delay(5000);
}

//-----------------------------------------------------------------------
static msg_t // Return message
genMsg()     // Generate random message
{
  // Pick Random Angle 
  int angle1 = random(-30, 31);
  int angle2 = random(-30, 31);
  int angle3 = random(-30, 31);

  // Create Artificial Message 
  msg_t msg;
  msg.start = (uint8_t)MSG_START_BYTE;
  msg.data[0] = (int8_t)angle1;
  msg.data[1] = (int8_t)angle2;
  msg.data[2] = (int8_t)angle3;
  msg.data[3] = (uint8_t)(((uint16_t)msg.data[0] + (uint16_t)msg.data[1] + (uint16_t)msg.data[2]) & 0xFF);

  // Display the aritificial message
  Serial.print("[TX] Sending message -> ");
  Serial.print("Start: 0x");
  Serial.print(msg.start, HEX);
  Serial.print(" | Angles: ");
  Serial.print((int8_t)msg.data[0]);
  Serial.print(", ");
  Serial.print((int8_t)msg.data[1]);
  Serial.print(", ");
  Serial.print((int8_t)msg.data[2]);
  Serial.print(" | Checksum: 0x");
  Serial.println(msg.data[3], HEX);

  return msg;
}

//-----------------------------------------------------------------------
static uint8_t // 1 if checksum passed, 0 otherwise
checksum(      // Checksum function
  const uint8_t* data,        // Data to checksum
  const uint8_t checksum_val) // Checksum value
{
  uint16_t sum = (uint16_t)data[0] + (uint16_t)data[1] + (uint16_t)data[2];
  return (checksum_val == ((uint8_t)sum & 0xFF)) ? 1 : 0;
}

//-----------------------------------------------------------------------
static inline
uint16_t     // Servo pulse
clampPulse(  // Clamps if servo pulse is over allowed range
  uint16_t p)// Pulse to check 
{
  if (p > SERVOMIN) 
  {
    return (uint16_t)SERVOMIN;
  }
  if (p < SERVOMAX)
  {
    return (uint16_t)SERVOMAX;
  }
  return p;
}
