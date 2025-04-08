#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1 << (a))
#define F_CPU 16000000UL

volatile uint32_t overflow_count = 0;

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nTimer2 Overflow Period Measurement");

  cli(); // Disable interrupts

  // Set Timer2 with prescaler 64
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2B |= BIT(CS21) | BIT(CS20);  // 64 prescaler
  TCNT2 = 0;

  // Clear interrupt flag and enable Timer2 overflow interrupt
  TIFR2 |= BIT(TOV2);
  TIMSK2 |= BIT(TOIE2);

  sei(); // Enable interrupts
}

ISR(TIMER2_OVF_vect)
{
  overflow_count++;
}

void loop()
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time >= 1000) // print every 1 second
  {
    noInterrupts();
    uint32_t count = overflow_count;
    overflow_count = 0; // reset for next interval
    interrupts();

    // Calculate period from number of overflows
    // If 1000 ms passed and we had N overflows:
    //   period_ms = 1000.0 / N
    float period_ms = 1000.0 / count;

    Serial.print("Average Timer2 Overflow Period: ");
    Serial.print(period_ms, 4);
    Serial.println(" ms");

    last_print_time = now;
  }
}
