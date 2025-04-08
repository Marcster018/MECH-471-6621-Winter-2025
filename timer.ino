
#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1UL << (a))

double my_micros();

// make t1_offset volatile so the compiler
// knows it's externally changed (by the interrupt)
// -- global variables accessed by interrupts should
// always be declared volatile in this manner
volatile float t1_offset = 0.0;

// t0 is used to sync micros() time with timer1 time
float t0;

void setup()
{  
	Serial.begin(115200);
  
	Serial.print("\nbegin program");
	delay(1000);
  
	cli();  // disable interrupts
  
	// clear timer1 control register (ie use default values)
	TCCR1A = 0;
	TCCR1B = 0;

	// for presaler of 64 the roll occurs every
	// T_roll = 64 / 16.0e6 * 65536.0 = 0.262144 s
	// -- could reduce prescaler by factor of (0.262144/0.01) = 26.2 
	// for a prescaler of 2.4 -- from the table the closest prescaler
	// no smaller than 2.4 is 8 -> use a prescaler of 8
	// -> T_roll = 8 / 16.0e6 * 65536.0 = 0.032768 s = 32.7 ms

	// set timer1 prescaler to 8
	TCCR1B |= BIT(CS11);
	
	// initialize timer1
	TCNT1 = 0;
	
	// clear previous overflow interrupts so an overflow interrupt doesn't 
	// immediately occur when enabled below
	TIFR1 |= BIT(TOV1);
	
	// set timer1 overflow interrupt bit in interrupt mask / register
	TIMSK1 = BIT(TOIE1);
	
	sei(); // enable interrupts
	
	// sync Arduino micros() time with timer1
	// -- set both initial times right beside each other
	
	// set timer1 initial time
	TCNT1 = 0; 
	
	// set micros() inital time
	t0 = micros()*1.0e-6; 
	
	// * note that micros should normally only be called when interrupts 
	// are enabled since it depends on timer0 overflow interrupt
	
	// by default interrupts are disabled inside an interrupt function
	// -- therefore, it's best not to call micros() inside interrupt functions
	// -- the same thing goes for my_micros()	
	
}


void loop()
{	
	float t1, t2;

	t1 = my_micros()*1.0e-6;
	
  	t2 = micros()*1.0e-6 - t0;
	
	Serial.print(t1,5);
	Serial.print(",");	
	
	Serial.print(t2,5);
	Serial.print(",");
	
	Serial.print( (t2 - t1)*1.0e6 );
	Serial.print("\n");
	
}


double my_micros()
{
	float t;
	
	t = TCNT1/16.0e6*8;
	
	// disable interrupts here in case t1_offset is changed
	// by the interrupt in the middle of t += t1_offset
	// -- in that case t1_offset might be only half updated
	
	cli();
	t += t1_offset; // add offset to clock time
	sei();
	
	return t*1.0e6;
}


ISR(TIMER1_OVF_vect)
{
	// increase offset by the roll time when the timer overflows
	t1_offset += 65536.0/16.0e6*8;
	
	// note the timer has 65536 counts when it rolls since 
	// the count goes from 0 to 65535 -- 65536 values	
}
