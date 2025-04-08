#include "dx.h"
#include <Arduino.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1 << (a))

// input u - V
void write_actuator(int16_t id, float u);

// ymin - minimum allowed value of position y (rad)
// ymax - maximum allowed value of position y (rad)
void set_range(int16_t id, float ymin, float ymax);

// position y - rad
// velocity v - rad/s
void read_sensor(int16_t id, float &y, float &v);

void stop_robot();

void activate_servo(int16_t id);

// conversion factors -- variables that depend on V_batt
float V_batt; // battery voltage V (at startup)
float count_to_V;  
float V_to_count;

void PID_controller1();

// you should normally try to minimize the number of global variables
// since too much sharing can get confusing
float t0; // initial time (s)

void setup() 
{	
	Serial.begin(115200);
	  
	Serial.print("\nprogram start\n");
	delay(3000);

	// Timer 0 interrupt configuration
	// Disable interrupts
	cli(); 
	
	// clear timer0 control register (ie use default values)
	TCCR2A = 0;
  	TCCR2B = 0;

	// set timer0 prescaler to 64
	TCCR2B |= BIT(CS22);
	
	// initialize timer
	TCNT2 = 0;

	// clear previous overflow interrupts
	TIFR2 |= BIT(TOV2);

	// set timer0 overflow interrupt bit in interrupt mask / register
	TIMSK2 = 0;
	TIMSK2 |= BIT(TOIE2);
	
	// enable interrupts
	sei(); 
	
	int16_t id, Temp;
	float t;
	char ch;
	
	// Arduino IDE serial monitor must be set to data rate below (2 Mbps)
	Serial.begin(2000000);
	
	// wait until Serial Monitor or save_serial program is started
	while( !Serial ) delay(1);
	
	Serial.print("\n% start program when 's' character is sent to the Serial");
	Serial.print("\n% input -- either Serial Monitor or save_serial.exe");	
	
	// note: the following code is the best way to start an Arduino 
	// program in my opinion since you can control the start time exactly
	// and you can wait until you are ready to watch the experiment.
	while(1) {
		if( Serial.available() > 0 ) { // is there incoming data
			ch = Serial.read(); // read one character / byte
			if( ch == 's' ) break;
		}
		delay(10); // leave some time for printing message above, etc.
	}
	
	// give some time for user to look at robot
	delay(3000);
	
	// activate dynamixel servos and function library
	activate_dx();  
	Serial.print("\n% starting dynamixel servos ...\n");
	
	// put servo into voltage control / PWM mode
	id = 1;
	activate_servo(id);
	
	// note V_batt is set in activate_servo
	Serial.print("\n% V_batt (V) = ");
	Serial.print(V_batt);
	
	get_temp(id,Temp); // deg C
	Serial.print("\n\n% Temp (deg C) = ");
	Serial.print(Temp);	
	
	Serial.print("\n\n");
	
	// might want to set reference position r equal to current 
	// position to prevent initial sudden movements
	
	// set servo position limits to shutdown the robot
	// if it gets out of control.
	// assume id is in the range 1 to 20
	// note: that 1.0 is the minimum safe limit even with reverse
	set_range(id,1.0,3.0);
	
	t0 = micros()*1.0e-6; // measure initial time (s)
	
	// run controller for 10 s
	while(t < 10) { 	
		// run the PID controller as fast as possible
		// hopefully will be 1 ms or less
		PID_controller1();		
		t = micros()*1.0e-6 - t0; // s		
	}
	
	// disable robot torque / controller to save battery
	stop_robot();
	
	// send '#' to Serial port to stop save_serial program
	Serial.print("#");
	
	// wait for the program to finish before termination
	delay(1000);
	
	// terminate the program
	exit(0); 
}


void PID_controller1()
{
	int16_t id, tick;
	float t, dt;
	float r, y, v, u;
	float kp, ki, kd;
	float e, ed, z, ei_max;
	
	// use static floats for local variables that need to be 
	// remembered between function calls.
	// static variables are like global variables that can only
	// be used by the function.
	// regular local variables get erased at the end of the function.
	static float tp = 0.0; // previous time (initial value = 0)
	static float ei = 0.0; // integral state (initial value = 0)
	static float ep = 0.0; // previous error
	
	// servo id
	id = 1;
	
	// PID controller gains
	kp = 50.0;
	ki = 20.0;
	kd = 3.0;
	
	// the control loop typically performs the following steps:
	// 1) read time
	// 2) read sensors
	// 3) calculate controller input
	// 4) write input to the actuators
	
	// 5) save data for plotting, etc.
	// -- be careful since this can slow the controller down
	// for the final controller don't do this
	// or maybe save it to SD card memory, etc.
		
	t = micros()*1.0e-6 - t0; // s
				
	read_sensor(id,y,v);
		
	// set reference input r(t) / desired y_des
	
	// NOTE: A = 3 is probably too big of step
	// -- will cause saturation
	if( t > 5 ) { // step input at t = 5 s
		r = 0.5; // A = 0.5
	} else {
		r = 0;
	}
		
	// calculate dt
	dt = t - tp; // measure sampling period dt
	tp = t; // save previous sample time

	// calculate controller error
	e  = r - y;
	ed = 0 - v; // ed = rd - yd

	// finite difference approximation for ed (FYI)
//	ed = (e - ep) / dt;
//	ep = e; // save current error for next time

	// standard integration I = ei
	ei += e*dt;	// I += e*dt

	// please try it without anti-windup logic first
	// for a large step and see what happens
	
	// then try it using anti-windup and compare

	// anti-windup / saturation integration logic ///////
		
	// set maximum integration bounds
	// ki*ei_max = ui_max = F*V_batt	
	// -> ei_max = F*V_batt/ki
	// where F*V_batt is a bit larger (1.5 x or so) than 
	// the static friction voltage
	if( ki > 0.0 ) {
		ei_max = 0.14*V_batt/ki; // want ki*ei < 0.14*V_batt
	} else {
		ei_max = 0.0;
	}
	
	// stop integration if integral is too large and increasing	
	// ei += z*dt
	// where z is set appropriately if integral is too large
	
	// set z
	if ( (ei > ei_max) && (e > 0) ) {
		// positive out of bounds, positive integration
		z = 0; // stop integration
	} else if ( (ei < -ei_max) && (e < 0) ) {
		// negative out of bounds, negative integration		
		z = 0; // stop integration
	} else { // either in bounds or integration reduces integral
		z = e; // normal integration
	}
		
	// NOTE: if you use this you must comment out ei += e*dt
	ei += z*dt;		
		
	////////////////////////////////////////////	

	// PID controller
	u = kp*e + ki*ei + kd*ed;

	// need to saturate the input u so it's not out of range 
	// otherwise the servo is automatically disabled
	
	// it's best to saturate in software rather than
	// rather than find out what happens if you
	// write the actuators out of range
	
	// NOTE: it's sometimes beneficial to make the limit
	// initially smaller so the robot (or you!) has less chance of 
	// getting damaged when you are first testing it
	
	if(u >  V_batt) u = V_batt;
	if(u < -V_batt) u = -V_batt;		
		
	write_actuator(id,u);		
		
	// print data to the serial port
		
	Serial.print(t,5); // s
	Serial.print(" ");	
	
	Serial.print(y,5); // rad
	Serial.print(" ");	

	Serial.print(r,5); // rad
	Serial.print(" ");			
		
	Serial.print(v,5); // rad/s
	Serial.print(" ");			
				
	Serial.print(e/PI*180,5); // deg
	Serial.print(" ");
				
	// NOTE: always plot u so you can check for saturation
	Serial.print(u); // V
	Serial.print(" ");
	
	// NOTE: ki*ei < 0.5*Vmax -- maybe even 0.2 or 0.3 instead of 0.5
	Serial.print(ki*ei,5); // ui (V)
	Serial.print(" ");
	
//	get_clock(id,tick);		
//	Serial.print(tick); // 1 count = 1 ms on dynamixel
		
	Serial.print("\n");

}

///////////////////////////////////////////////////////////
// the program below this line is less important to you
// -> focus on implementing your controller by modifying
// setup() and PID_controller1()
///////////////////////////////////////////////////////////

void loop() {
	// don't use loop
}

void write_actuator(int16_t id, float u)
{
	int16_t u_c;
	
	u_c = round(u * V_to_count); // count	
		
	// set the input voltage / PWM
	set_PWM(id,u_c);
}

// servo position limits
// assume id is in the range 1 to 20
int16_t yc_min[21] = {0}, yc_max[21] = {0};

void set_range(int16_t id, float ymin, float ymax)
{
	yc_min[id] = ymin * RAD_TO_COUNT;
	yc_max[id] = ymax * RAD_TO_COUNT;
}

void read_sensor(int16_t id, float &y, float &v)
{
	int16_t y_c, v_c;
	const int16_t UC_MAX = 885;
	
	// read data together -- faster and more
	// in sync vs separate read functions
	get_pv(id,y_c,v_c);
		
	// TODO: might want to check other servos
	// if out of bounds to prevent simultaneous errors
	// (unlikely but possible)
		
	// check if servo is out of bounds.
	// quickly decelerate and stop if that occurs
	if( ( y_c > yc_max[id] ) || ( y_c < yc_min[id] ) ) {
				
		if( y_c > yc_max[id] ) {
			// this input and delay time are well tuned to reverse
			// quickly with minimal overshoot beyond limits		
			set_PWM(id,-UC_MAX); // decelerate
			delay(100);	// give small time to decelerate
		}
		
		if( y_c < yc_min[id] ) {
			set_PWM(id,UC_MAX);
			delay(100);
		}
		
		stop_robot(); // turn off all motors
		
		Serial.print("\nrobot out of range error.");
		delay(100);
		
		exit(1); // end program
	}
	
	// unit conversions
	y = y_c * COUNT_TO_RAD; // rad
	v = v_c * COUNT_TO_RAD_PS; // rad/s
}

void stop_robot()
{
	int16_t id, u_c;

	id = 1; // servo of first joint

	// set the input to zero
	u_c = 0;
	set_PWM(id,u_c);
	delay(5);
	
	// turn off the motor
	disable_torque(id);
}

void activate_servo(int16_t id)
{
	int16_t V_batt_c, mode, u_c, r_c;
	
	// each connected dynamixel has it's own ID number
	// note: id = 254 will send to all servos (ie broadcast)
	
	// need to disable servo torque before changing control modes
	disable_torque(id);

	// change control mode
	// 1 - PI velocity control
	// 3 - PID position control (normal mode)
	// 4 - PID extended position control (-256 to 256 full turns)
	// 16 - PWM / voltage control (open loop / no feedback)
	
	// changing to position control resets the position
	// between 0 to 4095
	mode = 3;
	set_control_mode(id,mode);
	
	// set desired position equal to current position
	// to prevent initial sudden movements
	get_position(id,r_c);
	set_ref_position(id,r_c);	
	delay(5);
	
	// set to voltage control / PWM mode
	mode = 16;
	set_control_mode(id,mode);
	
	// enable torque before using other motion commands
	enable_torque(id);	

	// set PWM input to zero
	u_c = 0;
	set_PWM(id,u_c);
	delay(5);
	
	// read battery voltage and calculate default 
	// voltage unit conversion constants
	get_battery_voltage(id,V_batt_c);
	
	V_batt = V_batt_c * COUNT_TO_VBATT; // V	
	count_to_V = 1.0/885*V_batt;	
	V_to_count = 1.0/count_to_V;
	
}
