#ifndef DEFS_H_
#define DEFS_H_

#define F_CPU 16000000

/**
 * Controller wires:
 * Blue1: GND (for joystick Y)
 * Yellow: +5V (for joystick X)
 * Blue2 (White): GND (for joystick Y)
 * Green: +5V (for joystick Y)
 * Gray: GND (for trigger button)
 * White: TRIGGER_PORT
 * Orange: JOYSTICK_X_PORT
 * Violet: JOYSTICK_Y_PORT
 */

/**
 * NERF gun wires:
 * Red: Vin
 * Brown: GND
 * Green: SERVO_TRIGGER_PORT
 * Yellow: SERVO_YAW_PORT
 * White: SERVO_PITCH_PORT
 **/

typedef enum { MODE_JOYSTICK = 0, MODE_GYRO = 1 } ProgramMode;

/**** Pins ****/
typedef enum Port {
  CPB0 = 0,
  CPB1 = 1,
  CPB2 = 2,
  CPB3 = 3,
  CPB4 = 4,
  CPB5 = 5,
  CPB6 = 6,
  CPB7 = 7,
  CPB8 = 8,
  CPD0 = 9,
  CPD1 = 10,
  CPD2 = 11,
  CPD3 = 12,
  CPD4 = 13,
  CPD5 = 14,
  CPD6 = 15,
  CPD7 = 16,
  CPD8 = 17,
  CPC0 = 18,
  CPC1 = 19,
  CPC2 = 20,
  CPC3 = 21,
  CPC4 = 22,
  CPC5 = 23,
  CPC6 = 24,
  CPC7 = 25,
  CPC8 = 26
} Port;

typedef enum { LOW = 0, HIGH = 1 } Level;

typedef enum { INPUT = 0, OUTPUT = 1 } PinMode;

/** Analog Ports **/
const Port JOYSTICK_X_PORT = CPC1;
const Port JOYSTICK_Y_PORT = CPC2;
const Port GYRO_YAW_PORT = CPC3;
const Port GYRO_PITCH_PORT = CPC4;

/**** Digital Ports ****/
/** PORT B  **/
const Port TRIGGER_PORT = CPD2;
const Port TOGGLE_MODE_PORT = CPD7;
const Port MODE_LED_PORT = CPD4;

/** PORT D **/
const Port SERVO_YAW_PORT = CPB3;
const Port SERVO_TRIGGER_PORT = CPD3;
const Port SERVO_PITCH_PORT = CPD5;

/**** Other Constants ****/
/** Gyro **/
const double GYRO_VOLTAGE = 5;
const double GYRO_SENSITIVITY = 1.1;
const double GYRO_THRESHOLD = 0.18;
const int GYRO_YAW_LOW = -90;
const int GYRO_YAW_HIGH = 90;
const int GYRO_PITCH_LOW = -35;
const int GYRO_PITCH_HIGH = 15;

/** Joystick **/
const unsigned int JOYSTICK_LOW = 0;
const unsigned int JOYSTICK_HIGH = 1023;

/** Servos **/
const unsigned char SERVO_YAW_PWM_HIGH = 255;
const unsigned char SERVO_YAW_PWM_LOW = 128;
const unsigned char SERVO_PITCH_PWM_HIGH = 255;
const unsigned char SERVO_PITCH_PWM_LOW = 150;
const unsigned char SERVO_TRIGGER_PWM_PRESSED = 254;
const unsigned char SERVO_TRIGGER_PWM_UNPRESSED = 190;

/** Misc **/
const unsigned char LOOP_DELAY = 20; // Loop delay in ms

#endif // DEFS_H_
