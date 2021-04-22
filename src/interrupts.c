#include <avr/interrupt.h>

unsigned long timer0_millis = 0;
unsigned long timer0_frac = 0;

ISR(TIMER0_OVF_vect)
{
  // Every overflow is 1.024 ms
  timer0_millis++;
  // 1 frac unit is equivalent to 0.008 ms
  timer0_frac += 3;

  // 125*0.024 = 3
  if (timer0_frac >= 125) {
    timer0_frac -= 125;
    timer0_millis++;
  }
}

ISR(TIMER1_OVF_vect) { }

ISR(TIMER2_OVF_vect) { }
