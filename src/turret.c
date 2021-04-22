#include "defs.h"
#include "interrupts.c"
#include "util.c"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

unsigned int trigger_count = 0;

char trigger_on = 0;
ProgramMode program_mode = MODE_JOYSTICK;

double gyro_yaw_zero_voltage = 2.5;
double gyro_pitch_zero_voltage = 2.5;

double gyro_yaw = 0;
double gyro_pitch = 0;
double gyro_yaw_rate = 0;
double gyro_pitch_rate = 0;

int last_update_time = 0;

void initializePorts() {
  // Set Servo ports to output
  pinMode(SERVO_PITCH_PORT, OUTPUT);
  pinMode(SERVO_YAW_PORT, OUTPUT);
  pinMode(SERVO_TRIGGER_PORT, OUTPUT);

  // Set LED port to output
  pinMode(MODE_LED_PORT, OUTPUT);

  // Set trigger and mode button to input
  pinMode(TRIGGER_PORT, INPUT);
  pinMode(TOGGLE_MODE_PORT, INPUT);

  // Set gyro and joystick ports to input
  pinMode(GYRO_YAW_PORT, INPUT);
  pinMode(GYRO_PITCH_PORT, INPUT);
  pinMode(JOYSTICK_X_PORT, INPUT);
  pinMode(JOYSTICK_Y_PORT, INPUT);
}

void initializeTimers() {
  // Initialize timer0:
  // Phase correct non-inverted PWM with TOP 0xff and clk/64 prescalar
  // PWM Frequency: 490 Hz
  TCCR0A = (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);
  TCCR0B = (1 << CS00) | (1 << CS01);
  TIMSK0 = (1 << TOIE0); // Enable overflow interrupt

  // Initialize timer1:
  // Phase-correct, 10-bit non-inverted PWM with TOP 0x03ff and clk/8 prescalar
  // PWM Frequency: 980 Hz
  TCCR1A = (1 << WGM10) | (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << CS11);
  TIMSK1 = (1 << TOIE1); // Enable overflow interrupt

  // Initialize timer2:
  // Phase correct non-inverted PWM with TOP 0xff and clk/64 prescalar
  // PWM Frequency: 490 Hz
  TCCR2A = (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1);
  TCCR2B = (1 << CS22);
  TIMSK2 = (1 << TOIE2); // Enable overflow interrupt
}

void initializeADC() {
  // Use VCC reference voltage
  ADMUX = (1 << REFS0);
  // Enable ADC and use clk/128 prescalar
  ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
}

void calibrateGyro() {
  double yaw_sum = 0;
  double pitch_sum = 0;

  // Calculate sum of first 100 readings
  for (char i = 0; i < 100; i++) {
    yaw_sum += analogRead(GYRO_YAW_PORT);
    pitch_sum += analogRead(GYRO_PITCH_PORT);
    _delay_ms(10);
  }

  // Set average 0-velocity reference voltage
  gyro_pitch_zero_voltage = pitch_sum / 100.0 / 1023.0 * GYRO_VOLTAGE;
  gyro_yaw_zero_voltage = yaw_sum / 100.0 / 1023.0 * GYRO_VOLTAGE;

  // Reset Gyro
  gyro_pitch = 0;
  gyro_yaw = 0;
}

void updateGyro() {
  // Get mV equivalent of angular velocity
  gyro_pitch_rate = analogRead(GYRO_PITCH_PORT) / 1023.0 * GYRO_VOLTAGE -
                    gyro_pitch_zero_voltage;
  gyro_yaw_rate =
      analogRead(GYRO_YAW_PORT) / 1023.0 * GYRO_VOLTAGE - gyro_yaw_zero_voltage;

  // Get angular velocity in dps
  gyro_pitch_rate = gyro_pitch_rate / GYRO_SENSITIVITY;
  gyro_yaw_rate = gyro_yaw_rate / GYRO_SENSITIVITY;

  int delta = (millis() - last_update_time);

  // If within threshold, update gyro
  if (gyro_pitch_rate < -GYRO_THRESHOLD || gyro_pitch_rate > GYRO_THRESHOLD) {
    gyro_pitch += gyro_pitch_rate * delta;
  }

  if (gyro_yaw_rate < -GYRO_THRESHOLD || gyro_yaw_rate > GYRO_THRESHOLD) {
    gyro_yaw += gyro_yaw_rate * delta;
  }

  // Constrain gyro to 180 degrees of motion to avoid it growing out of bounds
  gyro_pitch = clamp(gyro_pitch, -90, 90);
  gyro_yaw = clamp(gyro_yaw, -90, 90);

  // Record time of last update
  last_update_time = millis();
}

void loop() {
  // Read inputs
  unsigned int joy_x = analogRead(JOYSTICK_X_PORT);
  unsigned int joy_y = analogRead(JOYSTICK_Y_PORT);
  Level press_trigger = digitalRead(TRIGGER_PORT);
  Level toggle_mode = digitalRead(TOGGLE_MODE_PORT);

  // Toggle program mode if button is pressed
  if (toggle_mode == LOW) {
    program_mode = !program_mode;
    _delay_ms(300);
  }

  // Trigger is on if not press_trigger
  trigger_on = !((unsigned char)press_trigger);

  // Update gyro value
  updateGyro();

  unsigned char yaw_pwm = SERVO_YAW_PWM_LOW;
  unsigned char pitch_pwm = SERVO_PITCH_PWM_LOW;
  unsigned char trigger_pwm = SERVO_TRIGGER_PWM_UNPRESSED;

  // Map joystick/gyro values to PWM servo values
  if (program_mode == MODE_JOYSTICK) {
    pitch_pwm = map(joy_y, JOYSTICK_LOW, JOYSTICK_HIGH, SERVO_PITCH_PWM_LOW,
                    SERVO_PITCH_PWM_HIGH);
    yaw_pwm = map(joy_x, JOYSTICK_LOW, JOYSTICK_HIGH, SERVO_PITCH_PWM_LOW,
                  SERVO_PITCH_PWM_HIGH);
    // Reverse direction
    yaw_pwm = SERVO_YAW_PWM_HIGH + SERVO_YAW_PWM_LOW - yaw_pwm;
  } else {
    pitch_pwm = map(gyro_pitch, GYRO_PITCH_LOW, GYRO_PITCH_HIGH,
                    SERVO_PITCH_PWM_LOW, SERVO_PITCH_PWM_HIGH);
    yaw_pwm = map(gyro_yaw, GYRO_YAW_LOW, GYRO_YAW_HIGH, SERVO_YAW_PWM_LOW,
                  SERVO_YAW_PWM_HIGH);
  }

  // Apply force on nerf trigger for 350ms and then release for 350ms
  if (trigger_on && trigger_count > 700) {
    trigger_pwm = SERVO_TRIGGER_PWM_PRESSED;
    analogWrite(SERVO_TRIGGER_PORT, trigger_pwm);
    trigger_count = 0;
  } else if (trigger_on && trigger_count > 350) {
    trigger_pwm = SERVO_TRIGGER_PWM_UNPRESSED;
    analogWrite(SERVO_TRIGGER_PORT, trigger_pwm);
    trigger_count += LOOP_DELAY;
  } else if (trigger_on) {
    trigger_count += LOOP_DELAY;
  } else {
    trigger_count = 0;
    trigger_pwm = SERVO_TRIGGER_PWM_UNPRESSED;
    analogWrite(SERVO_TRIGGER_PORT, trigger_pwm);
  }

  // Update PWM output on servos
  digitalWrite(MODE_LED_PORT, program_mode);
  analogWrite(SERVO_PITCH_PORT, pitch_pwm);
  analogWrite(SERVO_YAW_PORT, yaw_pwm);

  _delay_ms(LOOP_DELAY);
}

int main() {
  // Initialization
  initializePorts();
  initializeTimers();
  initializeADC();
  calibrateGyro();

  // Enable interrupts
  sei();

  // Loop forever
  while (1)
    loop();

  return 0;
}
