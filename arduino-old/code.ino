/**
 * Controller wires:
 * Blue1: GND (for joystick Y)
 * Yellow: +5V (for joystick X)
 * Blue2 (White): GND (for joystick Y)
 * Green: +5V (for joystick Y)
 * Gray: GND (for trigger button)
 * White: TRIGGER_PORT
 * Orange: JOYSTICK_Y_PORT
 * Violet: JOYSTICK_X_PORT
 */

/**
 * NERF gun wires:
 * Red: Vin
 * Brown: GND
 * Green: MOTOR_TRIGGER_PORT
 * Yellow: MOTOR_PITCH_PORT
 * White: MOTOR_YAW_PORT
**/

typedef enum MODE { MODE_JOYSTICK = 0, MODE_GYRO = 1};

const int JOYSTICK_X_PORT = A1;
const int JOYSTICK_Y_PORT = A2;
const int GYRO_YAW_PORT = A3;
const int GYRO_PITCH_PORT = A4;
const int TRIGGER_PORT = 2;
const int TOGGLE_MODE_PORT = 3;
const int MODE_LED_PORT = 4;

const int MOTOR_YAW_PORT = 9;
const int MOTOR_PITCH_PORT = 11;
const int MOTOR_TRIGGER_PORT = 10;

unsigned int trigger_count = 0;

bool trigger_on = false;
int program_mode = MODE_JOYSTICK;

double gyro_yaw_zero_voltage = 2.5;
double gyro_pitch_zero_voltage = 2.5;
const double GYRO_VOLTAGE = 5;
const double GYRO_SENSITIVITY = 1.1;
const double GYRO_THRESHOLD = 0.18;

double gyro_yaw = 0;
double gyro_pitch = 0;
double gyro_yaw_rate = 0;
double gyro_pitch_rate = 0;

float gyro_pitch_rate_correction = 0;
float gyro_yaw_rate_correction = 0;

int last_update_time = 0;

void update_gyro(int delay_ms) {
  gyro_pitch_rate = analogRead(GYRO_PITCH_PORT) / 1023.0 * GYRO_VOLTAGE - gyro_pitch_zero_voltage;
  gyro_yaw_rate = analogRead(GYRO_YAW_PORT) / 1023.0 * GYRO_VOLTAGE - gyro_yaw_zero_voltage;

  gyro_pitch_rate = gyro_pitch_rate / GYRO_SENSITIVITY;
  gyro_yaw_rate = gyro_yaw_rate / GYRO_SENSITIVITY;

  int delta = (millis() - last_update_time);

  if (gyro_pitch_rate < -GYRO_THRESHOLD || gyro_pitch_rate > GYRO_THRESHOLD) {
    gyro_pitch += gyro_pitch_rate * delta;
  }

  if (gyro_yaw_rate < -GYRO_THRESHOLD || gyro_yaw_rate > GYRO_THRESHOLD) {
    gyro_yaw += gyro_yaw_rate * delta;
  }

  if (gyro_pitch > 90) gyro_pitch = 90;
  else if (gyro_pitch < -90) gyro_pitch = -90;

  if (gyro_yaw > 90) gyro_yaw = 90;
  else if (gyro_yaw < -90) gyro_yaw = -90;

  last_update_time = millis();
}

void calibrate_gyro() {
  double yaw_sum = 0;
  double pitch_sum = 0;
  for (int i = 0; i < 100; i++) {
    yaw_sum += analogRead(GYRO_YAW_PORT);
    pitch_sum += analogRead(GYRO_PITCH_PORT);
    delay(10);
  }

  gyro_pitch_zero_voltage = pitch_sum / 100.0 / 1023.0 * GYRO_VOLTAGE;
  gyro_yaw_zero_voltage = yaw_sum / 100.0 / 1023.0 * GYRO_VOLTAGE;


  //gyro_yaw_rate_correction = yaw_rate_sum / 100;
  //gyro_pitch_rate_correction = pitch_rate_sum / 100;

  gyro_pitch = 0;
  gyro_yaw = 0;
}

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_YAW_PORT, OUTPUT);
  pinMode(MOTOR_PITCH_PORT, OUTPUT);
  pinMode(MOTOR_TRIGGER_PORT, OUTPUT);
  pinMode(MODE_LED_PORT, OUTPUT);

  pinMode(TRIGGER_PORT, INPUT);
  pinMode(TOGGLE_MODE_PORT, INPUT);

  calibrate_gyro();
}

void loop() {
  int joy_x = analogRead(JOYSTICK_X_PORT);
  int joy_y = analogRead(JOYSTICK_Y_PORT);
  bool press_trigger = digitalRead(TRIGGER_PORT);
  bool toggle_mode = digitalRead(TOGGLE_MODE_PORT);

  if (toggle_mode == LOW) {
    program_mode = !program_mode;
    delay(500);
  }

  trigger_on = !press_trigger;

  unsigned char yaw_pwm, pitch_pwm, trigger_pwm;

  if (program_mode == MODE_JOYSTICK) {
    pitch_pwm = map(joy_y, 0, 1023, 128, 255);
    yaw_pwm = map(joy_x, 0, 1023, 128, 255);
    // Reverse direction
    pitch_pwm = 255 + 128 - pitch_pwm;
    yaw_pwm = 255 + 128 - yaw_pwm;
  } else {
    pitch_pwm = map(gyro_pitch, -90, 90, 128, 255);
    yaw_pwm = map(gyro_yaw, -35, 15, 128, 255);
    // Reverse direction
    yaw_pwm = 255 + 128 - yaw_pwm;
    pitch_pwm = 255 + 128 - pitch_pwm;
  }


  update_gyro(10);

  /*if (press_trigger == LOW) {
    trigger_pwm = 254;
  } else {
    trigger_pwm = 190;
  }*/


  /*if (press_trigger == LOW) {
    trigger_on = !trigger_on;
    delay(500);
  }*/

  if (trigger_on && trigger_count > 500) {
    trigger_pwm = 254;
    trigger_count = 0;
  } else if (trigger_on && trigger_count > 250){
    trigger_pwm = 190;
    trigger_count += 10;

  } else if (trigger_on) {
    trigger_count += 10;
  } else  {
    trigger_count = 0;
    trigger_pwm = 190;
  }

  digitalWrite(MODE_LED_PORT, (int) program_mode);
  analogWrite(MOTOR_PITCH_PORT, pitch_pwm);
  analogWrite(MOTOR_YAW_PORT, yaw_pwm);
  analogWrite(MOTOR_TRIGGER_PORT, trigger_pwm);


  /*
  Serial.print("Mode: ");
  Serial.print((int)program_mode);
  Serial.print("Joystick X: ");
  Serial.print(joy_x);
  Serial.print("\tJoystick Y: ");
  Serial.print(joy_y);
  Serial.print("\tGyro yaw: ");
  Serial.print(gyro_yaw);
  Serial.print("\tGyro pitch: ");
  Serial.print(gyro_pitch);
  Serial.print("\t Yaw: ");
  Serial.print(yaw_pwm);
  Serial.print("\t Pitch: ");
  Serial.print(pitch_pwm);
  Serial.print("\t Trigger: ");
  Serial.print(press_trigger);
  Serial.print("\t trigger_count: ");
  Serial.println(trigger_count);
  */

  delay(10);
}
