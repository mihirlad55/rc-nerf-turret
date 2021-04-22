#include "util.h"
#include <avr/io.h>

extern unsigned long timer0_millis;

int map(int val, int from_low, int from_high, int to_low, int to_high) {
  int from_delta = from_high - from_low;
  int to_delta = to_high - to_low;

  // Get ratio of value relative to starting range
  float ratio = (val - from_low) / (float) from_delta;

  // Convert value to target range
  int mapped = to_low + ratio * to_delta;

  // Make sure value is still within target range
  if (val < from_low)
    return to_low;
  else if (val > from_high)
    return to_high;

  return mapped;
}

int clamp(int val, int low, int high) {
  // If value is out of range, set value to bound
  if (val < low)
    return low;
  else if (val > high)
    return high;

  return val;
}

unsigned long millis() {
  // Return timer0's count of milliseconds
  return timer0_millis;
}

unsigned char portToBit(Port port) {
  switch (port) {
    case CPB0:
    case CPC0:
    case CPD0:
      return 0;

    case CPB1:
    case CPC1:
    case CPD1:
      return 1;

    case CPB2:
    case CPC2:
    case CPD2:
      return 2;

    case CPB3:
    case CPC3:
    case CPD3:
      return 3;

    case CPB4:
    case CPC4:
    case CPD4:
      return 4;

    case CPB5:
    case CPC5:
    case CPD5:
      return 5;

    case CPB6:
    case CPC6:
    case CPD6:
      return 6;

    case CPB7:
    case CPC7:
    case CPD7:
      return 7;

    case CPB8:
    case CPC8:
    case CPD8:
      return 8;

    default:
      return -1;
  }
}

void digitalWrite(const Port port, const Level level) {
  unsigned char bit = portToBit(port);

  switch (port) {
    case CPB0:
    case CPB1:
    case CPB2:
    case CPB3:
    case CPB4:
    case CPB5:
    case CPB6:
    case CPB7:
    case CPB8:
      if (level == LOW)
        PORTB &= ~(1 << bit);
      else if (level == HIGH)
        PORTB |= (1 << bit);
      return;
    case CPD0:
    case CPD1:
    case CPD2:
    case CPD3:
    case CPD4:
    case CPD5:
    case CPD6:
    case CPD7:
    case CPD8:
      if (level == LOW)
        PORTD &= ~(1 << bit);
      else if (level == HIGH)
        PORTD |= (1 << bit);
      return;
    default:
      return;
  }
}

Level digitalRead(const Port port) {
  switch (port) {
    case CPB0:
      return (Port) (PINB & (1 << 0));
    case CPB1:
      return (Port) (PINB & (1 << 1));
    case CPB2:
      return (Port) (PINB & (1 << 2));
    case CPB3:
      return (Port) (PINB & (1 << 3));
    case CPB4:
      return (Port) (PINB & (1 << 4));
    case CPB5:
      return (Port) (PINB & (1 << 5));
    case CPB6:
      return (Port) (PINB & (1 << 6));
    case CPB7:
      return (Port) (PINB & (1 << 7));
    case CPB8:
      return (Port) (PINB & (1 << 8));
    case CPD0:
      return (Port) (PIND & (1 << 0));
    case CPD1:
      return (Port) (PIND & (1 << 1));
    case CPD2:
      return (Port) (PIND & (1 << 2));
    case CPD3:
      return (Port) (PIND & (1 << 3));
    case CPD4:
      return (Port) (PIND & (1 << 4));
    case CPD5:
      return (Port) (PIND & (1 << 5));
    case CPD6:
      return (Port) (PIND & (1 << 6));
    case CPD7:
      return (Port) (PIND & (1 << 7));
    case CPD8:
      return (Port) (PIND & (1 << 8));
    default:
      return LOW;
  }
}

void analogWrite(Port port, unsigned char pwm) {
  switch (port) {
    case CPB1:
      OCR1A = pwm;
      return;
    case CPB2:
      OCR1B = pwm;
      return;
    case CPB3:
      OCR2A = pwm;
      return;
    case CPD3:
      OCR2B = pwm;
      return;
    case CPD5:
      OCR0B = pwm;
      return;
    case CPD6:
      OCR0A = pwm;
      return;
    default:
      return;
  }
}

void pinMode(Port port, PinMode mode) {
  switch (port) {
    case CPB0:
    case CPB1:
    case CPB2:
    case CPB3:
    case CPB4:
    case CPB5:
    case CPB6:
    case CPB7:
    case CPB8:
      if (mode == OUTPUT)
        DDRB |= (1 << portToBit(port));
      else if (mode == INPUT)
        DDRB &= ~(1 << portToBit(port));
      return;

    case CPC0:
    case CPC1:
    case CPC2:
    case CPC3:
    case CPC4:
    case CPC5:
    case CPC6:
    case CPC7:
    case CPC8:
      if (mode == OUTPUT)
        DDRC |= (1 << portToBit(port));
      else if (mode == INPUT)
        DDRC &= ~(1 << portToBit(port));
    return;

    case CPD0:
    case CPD1:
    case CPD2:
    case CPD3:
    case CPD4:
    case CPD5:
    case CPD6:
    case CPD7:
    case CPD8:
      if (mode == OUTPUT)
        DDRD |= (1 << portToBit(port));
      else if (mode == INPUT)
        DDRD &= ~(1 << portToBit(port));
    return;
  }
}

unsigned int analogRead(Port port) {
  // Select analog port
  ADMUX = portToBit(port) | (1 << REFS0);

  // Start conversion
  ADCSRA |= (1 << ADSC);

  // Wait for conversion
  while ((ADCSRA & (1 << ADIF)) == 0) ;

  // Clear ADIF flag
  ADCSRA |= (1 << ADIF);

  return ADC;
}
