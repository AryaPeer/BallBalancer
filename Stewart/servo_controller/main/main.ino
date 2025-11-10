/**
 ********************************************************************************
 * @file main.ino
 * @author MTE380-Group  
 * @brief   
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/************************************
 * MACROS AND DEFINES
 ************************************/
#define SERVOMIN  350 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  250 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMID  300
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define NUM_SERVOS  3 
#define MSG_START_BYTE 0xAA
#define MSG_LENGTH 5

/************************************
 * TYPEDEFS
 ************************************/
 typedef struct msg
 {
  uint8_t start;
  uint8_t angle1;
  uint8_t angle2;
  uint8_t angle3;
  uint8_t checksum;
 } msg;

/************************************
 * Static Variables
 ************************************/

 /************************************
 * Static Functions
 ************************************/
static uint8_t checksum(const uint8_t* data);
static inline uint16_t clampPulse(int32_t p);

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting Servo Controller Setup");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.setPWM(0, 0, SERVOMID);
  pwm.setPWM(1, 0, SERVOMID);
  pwm.setPWM(2, 0, SERVOMID);

  delay(10);
}

//-----------------------------------------------------------------------
void loop() 
{
  // Receive message
  if (Serial.available() > 0)
  {
    // Wait for message
    uint8_t recv_byte = Serial.read();
    uint8_t data[MSG_LENGTH - 1]; //Angles and checksum

    // If new motor command is sent, process next three bytes
    if (recv_byte == MSG_START_BYTE)
    {
      int bytes_read = Serial.readBytes(data, MSG_LENGTH-1);

      // Validate message
      if ( (bytes_read == MSG_LENGTH-1) && (checksum(data) == 1) )
      {
        // Interpret as signed angles (two’s complement)
        int8_t a1 = (int8_t)data[0];
        int8_t a2 = (int8_t)data[1];
        int8_t a3 = (int8_t)data[2];

        // Pulse = SERVO_MID + (-2.5)*angle  (integer math, no float)
        int32_t p1 = (int32_t)SERVOMID + (int32_t)(-25 * a1) / 10;
        int32_t p2 = (int32_t)SERVOMID + (int32_t)(-25 * a2) / 10;
        int32_t p3 = (int32_t)SERVOMID + (int32_t)(-25 * a3) / 10;
        
        uint16_t m1_pulse = clampPulse(p1);
        uint16_t m2_pulse = clampPulse(p2);
        uint16_t m3_pulse = clampPulse(p3);

        pwm.setPWM(0, 0, m1_pulse);
        pwm.setPWM(1, 0, m2_pulse);
        pwm.setPWM(2, 0, m3_pulse);
      }
    }
    
  }
  delay(10);
}

//-----------------------------------------------------------------------
static 
uint8_t 
checksum( 
  const uint8_t* data)
{
  return (data[3] == (data[0] + data[1] + data[2]) & 0xFF);
}

//-----------------------------------------------------------------------
static inline 
uint16_t
clampPulse(
  int32_t p) 
{
  if (p > SERVOMIN) 
  {
    return (uint16_t)SERVOMIN;
  }
  if (p < SERVOMAX)
  {
    return (uint16_t)SERVOMAX;
  }
  return (uint16_t)p;
}