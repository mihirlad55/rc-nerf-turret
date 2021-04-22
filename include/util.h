#ifndef UTIL_H_
#define UTIL_H_

/**
 * Map a range of values onto another range of values
 *
 * @param val The value to map
 * @param from_low The minimum value of the starting range
 * @param from_high The minimum value of the starting range
 * @param to_low The minimum value of the target range
 * @param to_high The maximum value of the target range
 *
 * @return a value proportional to its original range in the target range, but
 * constrained by the target range
 */
int map(int val, int from_low, int from_high, int to_low, int to_high);

/**
 * Return value within a constrained range
 *
 * @param val The value to constrain
 * @param low THe minimum this value can be
 * @param high The maximum this value can be
 *
 * @return The constrained value
 */
int clamp(int val, int low, int high);

/**
 * The number of milliseconds the arduino has been up
 */
unsigned long millis();

/**
 * Convert the Port to the bit index of its associated port
 *
 * @param port The port to convert
 *
 * @return A number between 0-7 representing the bit of the port
 */
unsigned char portToBit(Port port);

/**
 * Write a PWM output to a PWM port
 *
 * @param port The PWM port. This can only be CPB1, CPB2, CPB3, CPD3, CPD5, or
 *             CPD6
 * @param pwm A value ranging from 0-255 representing the duty cycle of the PWM
 */
void analogWrite(Port port, unsigned char pwm);

/**
 * Set a port's digital output
 *
 * @param port The port whose output to set
 * @param level Can be HIGH or LOW
 */
void digitalWrite(const Port port, const Level level);

/**
 * Set a port's pin to output mode or input mode
 *
 * @param port The port whose direction to set
 * @param mode Can be OUTPUT or INPUT
 */
void pinMode(Port port, PinMode mode);

/**
 * Read a port's input
 *
 * @param port The port to read
 *
 * @return Can be HIGH or LOW
 */
Level digitalRead(const Port port);

/**
 * Read an analog port's input
 *
 * @param The port to read
 *
 * @return A value between 0-1023 in reference to VCC
 */
unsigned int analogRead(Port port);

#endif // UTIL_H_
