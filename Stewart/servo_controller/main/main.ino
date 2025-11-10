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
 typedef struct msg_t
 {
  uint8_t start;
  uint8_t data[4];
 } msg_t;

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
    msg_t msg;
    msg.start = Serial.read();

    // If new motor command is sent, process next three bytes
    if (msg.start == MSG_START_BYTE)
    {
      int bytes_read = Serial.readBytes(msg.data, MSG_LENGTH - 1);

      // Validate message
      if ( (bytes_read == MSG_LENGTH - 1)) //&& checksum(msg.data))
      {
        // Interpret as signed angles (two’s complement)
        int8_t raw_a1 = (int8_t)msg.data[0];
        int8_t raw_a2 = (int8_t)msg.data[1];
        int8_t raw_a3 = (int8_t)msg.data[2];

        // Pulse = SERVO_MID + (-2.5)*angle  (integer math, no float)
        uint16_t m1_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a1) / 10);
        uint16_t m2_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a2) / 10);
        uint16_t m3_pulse = clampPulse((int32_t)SERVOMID + (int32_t)(-25 * raw_a3) / 10);

        pwm.setPWM(0, 0, m1_pulse);
        pwm.setPWM(1, 0, m2_pulse);
        pwm.setPWM(2, 0, m3_pulse);
      }
    }
    
  }
  delay(10);
}

//-----------------------------------------------------------------------
static uint8_t 
checksum( 
  const uint8_t* data)
{
  uint16_t sum = (uint16_t)data[0] + (uint16_t)data[1] + (uint16_t)data[2];
  return (uint8_t)(data[3] == (uint8_t)((sum) & 0xFF));
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
