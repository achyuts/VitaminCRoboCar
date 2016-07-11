//  BOE Bot Dead Reckoning using wheel encoders
// with IR beacon and HC-SR04 distance sensing
#include <Servo.h>
#include <math.h>
// Hacked version of the NewPing library is BotPing
#include <BotPing.h>

// Debug Pins
#define ZERO_SERVOS 6
#define HALT_TURRET 7

// Pinger 
#define TRIGGER_PIN  9  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN     10  // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping in cm.
volatile char ping_in_progress = 0;
volatile char isr_interrupt_count = 0;
volatile char isr_preamble_count = 0;
volatile char isr_poll_bits = 0;
volatile char isr_input_pin = 0;
volatile char isr_ir_value = 0;
volatile char isr_ir_available = 0;
unsigned long us_ping_result;
BotPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Infrared Beacon Receiver
#define IR_RX_PIN 5
volatile uint8_t *irrxInput;
uint8_t irrxBit;


// Switch to enable transmission of telemetry data over serial port
// Otherwise human readable data is printed
char bot_plot = 0;

// Analog inputs used for wheel encoder photo transistors
#define P_Right_Encoder 4
#define P_Left_Encoder 5

// Motor servo control
#define P_Right_Servo 12
#define P_Left_Servo 13
#define L_SERVO_CENTER 1500
#define R_SERVO_CENTER 1500

// Scanner Servo
#define P_Scanner_Servo 11
int scanner_angle = 90;
int old_angle = 90;
int scanner_increment = 9;
int scanner_offset = 0;

// Goal Seeking PID Variables ---------------------------------------
struct go_to_st {
	double gx;
	double gy;
	int speed;
	double tolerance;
	int last;
};
PROGMEM const struct go_to_st go_to_table[] = {
#include "robot_goals.h"
};

// Next point in FLASH table is copied to RAM here
struct go_to_st go_to_point;

char go_to_done = 1;
#define GO_TO_KP 30.0
#define GO_TO_KI 0.0
#define GO_TO_KD 0.0
#define TOO_MUCH_ERROR 50.0
double go_to_x = 0.0;
double go_to_y = 0.0;
double go_to_PID = 0.0;
double go_to_old_err = 0.0;
double go_to_err;
double go_to_distance;
double go_to_tolerance;
int go_to_speed;
double goal_heading, bot_heading, Pterm, Iterm, Dterm;
int right_speed, left_speed;
int go_to_state;
// end of goal seeking variables -----------------------------------

//////// FSM variables
unsigned long us_last_ping;
int stop_distance;

// Odometry constants YOU MUST CALIBRATE THESE CONSTANTS AGAINST YOUR ROBOT
// TICKS_PER_CIRCLE is the number of encoder ticks needed on one wheel to
// turn the robot 360 degrees when the other wheel is not moving
// #define TICKS_PER_CIRCLE 99.6
//#define TICKS_PER_CIRCLE 102
#define TICKS_PER_CIRCLE 153
// DELTA is the amount the robot turns (radians) when one wheel turns by one tick
// delta = 2*PI/TICKS_PER_CIRCLE
//#define DELTA 0.0615998559527410438914243800643
#define DELTA 0.0410665706351606959276162533762

// SIN_DELTA and COS_DELTA are the sine and cosine of DELTA
//#define SIN_DELTA 0.06156090613394283745053467245961
#define SIN_DELTA 0.04105502873160330941527811742663
//#define COS_DELTA 0.99810332873704407815955807227985
#define COS_DELTA 0.99915688688806385405335903139591
// L_SIN_DELTA is the distance the robot moves forward
// when one wheel turns one tick forward
//#define L_SIN_DELTA 0.125
#define L_SIN_DELTA 0.08333333333333333333333333333333

// Odometry variables
// See Parallax application note ApplyEncoder.pdf
// The angle of the robot is not stored as theta, but as sin(theta)
// and cos(theta). No sine and cosine function calls are made in this
// program. The sine and cosine valuse are updated directly.
#define SIDE_LEFT 1
#define SIDE_RIGHT 0
double bot_x;
double bot_y;
double bot_sin;
double bot_cos;
char NewSequence;
char LastSide;

// Servos
Servo servoLeft;
Servo servoRight;
Servo servoScanner;

// Encoder counts updated by poll_encoders() Polling Routine
volatile long ECount_Left;
volatile long ECount_Right;
volatile long WCount_Left;
volatile long WCount_Right;

char ENC_Right, Old_ENC_Right;
char ENC_Left, Old_ENC_Left;

// Servo motor current direction and power
// Forward-Reverse: 1=Forward, 0=stationary, -1=Reverse
#define FR_FORWARD 1
#define FR_REVERSE -1
#define FR_STATIONARY 0
volatile char Fwd_Rev_Left;
volatile char Fwd_Rev_Right;

// Power: -100...0...+100
int Power_Left;
int Power_Right;

// Time allocation in microsoeconds for base level loop
#define BASE_TIME 10000
unsigned long us_consumed_time;	// Loop time consumed by control
unsigned long max_consumed_time = 0;
// System microsecond tick markers
unsigned long us_ticks;
unsigned long us_start_marker;

// Counter for inline tasks executed less often than once per loop
// Every loop takes 10 msec, so if EVERY_N_LOOPS is 10 then tasks
// in the section in loop() get called every 100 msec.
#define EVERY_N_LOOPS 10
#define MIDDLE_LOOP 5
unsigned char loop_counter_n;
// every second
long loop_counter_s;

void setup()
{
	pinMode(ZERO_SERVOS, INPUT_PULLUP);
	pinMode(HALT_TURRET, INPUT_PULLUP);
	Serial.begin(115200);

	servoLeft.attach(P_Left_Servo);
	servoRight.attach(P_Right_Servo);
	servoScanner.attach(P_Scanner_Servo);
	delay(100);
	servoScanner.write(scanner_angle + scanner_offset);
	delay(300);
	//if (digitalRead(ZERO_SERVOS) == LOW) {
	//	servoLeft.writeMicroseconds(L_SERVO_CENTER);
	//	servoRight.writeMicroseconds(R_SERVO_CENTER);
	//	while (1)
	//		;
	//}
	// Serial.println("Align wheels");
	align_wheels();
	delay(1000);
	ENC_Right = 0;
	ENC_Left = 0;
	Old_ENC_Right = 0;
	Old_ENC_Left = 0;
	poll_encoders();
	poll_encoders();
	poll_encoders();
	
	reset_encoder_counts();
	
	loop_counter_n = 2;
	loop_counter_s = 0;
	
	// Initialize odometry
	delay(1000);
	// Serial.println("Init Odometer");
	reset_odometry();

	// Initialize GO TO controller
	go_to_state = 0;
	// Serial.println("Init GOTO");
	delay(100);
	////////go_to_goal_idx(go_to_state);
	// Serial.println("GOTO Done");

	// Initial time
	us_ticks = micros();
	us_start_marker = us_ticks;
	
	// Enable Ping, IR and Periodic Interrupts
	irrxInput = portInputRegister(digitalPinToPort(IR_RX_PIN));
	irrxBit = digitalPinToBitMask(IR_RX_PIN);
	// These use the HACKED NewPing Library called BotPing
	// BotPing::ping_timer() simply triggers a ping and does not call BotPing::timer_us
	// BotPing::check_timer() checks for an echo but does not stop the timer
	// The interrupt handler registered by BotPing::sonar.timer_us() can then 
	//   handle both echo and IR processing
	sonar.ping_timer(periodicInterrupt);	// 
	ping_in_progress = 1;
	sonar.timer_us(ECHO_TIMER_FREQ, periodicInterrupt);
	
	// move two feet forward and stop
	go_to_goal(0.0, 24.0, 30, 2.0);
	stop_distance = 6;
}  
 
void loop()
{
	// poll encoders and perform odometry
	poll_encoders();

	// Invoke GO TO Controller
	go_to_goal_PID();
	// If goal is reached advance to the next goal, if any left
	////////if (go_to_done) {
	////////	if (go_to_point.last == 0) {
	////////		go_to_state++;
	////////		go_to_goal_idx(go_to_state);
	////////	}
	////////}		
	
	loop_counter_n++;
	if (loop_counter_n == EVERY_N_LOOPS) {
		loop_counter_n = 0;
		// less often called tasks here (10 times a second)
		// Navigation, response to sensor stimulus and so on can happen here

		// Add your 100 mS tasks here
		Serial.print(old_angle);
		Serial.print(",");
		if (ping_in_progress == 1) {
			// Echo should have come back or timed out by now
			// if not then Pinger Error
			Serial.print("0,");
			us_last_ping = ((unsigned long)60)*((unsigned long)US_ROUNDTRIP_IN);
		} else {
			// Ping came back
			// Latest echo delay is in variable: us_ping_result
			us_last_ping = us_ping_result;
			Serial.print(us_ping_result/US_ROUNDTRIP_IN);
			Serial.print(",");
		}
		us_ping_result = 0;
		// Send another ping 
		sonar.ping_timer(periodicInterrupt);
		ping_in_progress = 1;
		// Check for IR Beacon
		if (isr_ir_available) {
			isr_ir_available = 0;
			Serial.println(isr_ir_value&0x0f);
		} else {
			Serial.println("0");
		}

		//////// 10x a second
		scanner_angle = 90;
		if (us_last_ping/US_ROUNDTRIP_IN < stop_distance) {
			go_to_done = 1;
			set_speeds_calibrated(0, 0);
			go_to_goal(bot_x+12.0, bot_y, 30, 2.0);
			stop_distance = 0;
		}
		
		// Tasks done even less often (2x a second)
		// Stagger prnting tasks over two different loop iterations
		loop_counter_s++;
		if ((loop_counter_s % 5) == 0) {
			// if (bot_plot) print_debug1a(); else print_debug1();
			loop_counter_s = 0;
		}
		if ((loop_counter_s % 5) == 2) {
			// if (bot_plot) print_debug2a(); else print_debug2();
		}
		// Update the scanner angle but don't move the servo yet
		////////old_angle = scanner_angle;
		////////scanner_angle += scanner_increment;
		////////if (scanner_angle > 171) {
		////////	scanner_angle = 171 - scanner_increment;
		////////	scanner_increment = - scanner_increment;
		////////} else if (scanner_angle < 9) {
		////////	scanner_angle = 9 - scanner_increment;
		////////	scanner_increment = -scanner_increment;
		////////}
	}
	// Move scanner servo at a time staggered 50 ms from ping
	if (loop_counter_n == MIDDLE_LOOP) {
		if (digitalRead(HALT_TURRET) != LOW) {
			servoScanner.write(scanner_angle + scanner_offset);
		}
	}

	// Whatever real time is left in loop() can be used for other tasks
	// max_consumed_time is used to record maximum realtime used per loop
	us_consumed_time = micros() - us_start_marker;
	if (max_consumed_time < us_consumed_time)
		max_consumed_time = us_consumed_time;
	if (us_consumed_time < BASE_TIME) {
		while((micros() - us_start_marker) < BASE_TIME)
			do_comm_tasks();
	}
	us_start_marker = micros();
}

void do_comm_tasks()
{
}

void print_debug1()
{
	Serial.print("PID=");
	Serial.print(go_to_PID,2);
	Serial.print("  D=");
	Serial.print(go_to_distance, 2);
	Serial.print("  x=");
	Serial.print(bot_x, 2);
	Serial.print("  y=");
	Serial.print(bot_y, 2);
	Serial.print("  a=");
	Serial.print(bot_heading*(180/M_PI), 2);
}
void print_debug1a()
{
	Serial.print(bot_x, 2);
	Serial.print(",");
	Serial.print(bot_y, 2);
	Serial.print(",");
	Serial.print(bot_heading*(180/M_PI), 2);
	Serial.print(",");
}
void print_debug2()
{
	
	Serial.print("  maxtime=");
	Serial.println(max_consumed_time);
}
void print_debug2a()
{
	Serial.print(go_to_x, 2);
	Serial.print(",");
	Serial.print(go_to_y, 2);
	Serial.print(",");
	Serial.print(left_speed);
	Serial.print(",");
	Serial.println(right_speed);
}

// Read the wheel encoders and perform odometry
// This routine would have been much simpler if crab-walk compensation was not necessary
// Need a volunteer to simplify this code
void poll_encoders()
{
	double bcos;
	
	// Read the wheel sensor values. This blows away 200 microseconds
	int VR = analogRead(P_Right_Encoder);
	int VL = analogRead(P_Left_Encoder);

	// Check current phototransistor levels with hysteresis
	// ADC values 300 and 600 correspond to the Vil max, and Vih min of the ATMega
	if (VR < 300) ENC_Right = 0;
	else if (VR > 600) ENC_Right = 1;
	if (VL < 300) ENC_Left = 0;
	else if (VL > 600) ENC_Left = 1;
	
	// Process the right wheel first
	if (ENC_Right != Old_ENC_Right) {
		if (Fwd_Rev_Right > 0)
			ECount_Right++;
		else if (Fwd_Rev_Right < 0)
			ECount_Right--;
		Old_ENC_Right = ENC_Right;
		// Odometry
		if (NewSequence == 0) {	// If last step was the 2nd of a sequence start a new one
			NewSequence = 1;
		} else {	// If side is same as last one start a new sequence
			if (LastSide == SIDE_RIGHT) NewSequence = 1; else NewSequence = 0;
			// But reverse the roles of the wheels if going backward
			if (Fwd_Rev_Right == FR_REVERSE) NewSequence ^= 1;	
		}
		// Remember that this side moved (but switch wheel roles if going backward)
		if (Fwd_Rev_Right == FR_REVERSE) LastSide = SIDE_LEFT; else LastSide = SIDE_RIGHT;

		if (NewSequence == 1) {
			// Update position first
			if (Fwd_Rev_Right != FR_REVERSE) {
				bot_x = bot_x + bot_cos * L_SIN_DELTA;
				bot_y = bot_y + bot_sin * L_SIN_DELTA;
			} else {
				bot_x = bot_x - bot_cos * L_SIN_DELTA;
				bot_y = bot_y - bot_sin * L_SIN_DELTA;
			}
			// Then update angle
			if (LastSide == SIDE_RIGHT) {
				bcos = (bot_cos * COS_DELTA) - (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) + (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			} else {
				bcos = (bot_cos * COS_DELTA) + (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) - (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			}

		} else {
			// Update angle first
			if (LastSide == SIDE_RIGHT) {
				bcos = (bot_cos * COS_DELTA) - (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) + (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			} else {
				bcos = (bot_cos * COS_DELTA) + (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) - (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			}
			// Then update position
			if (Fwd_Rev_Right != FR_REVERSE) {
				bot_x = bot_x + bot_cos * L_SIN_DELTA;
				bot_y = bot_y + bot_sin * L_SIN_DELTA;
			} else {
				bot_x = bot_x - bot_cos * L_SIN_DELTA;
				bot_y = bot_y - bot_sin * L_SIN_DELTA;
			}
		}

	}
	
	// Process the left wheel
	if (ENC_Left != Old_ENC_Left) {
		if (Fwd_Rev_Left > 0)
			ECount_Left++;
		else if (Fwd_Rev_Left < 0)
			ECount_Left--;
		Old_ENC_Left = ENC_Left;
		// Odometry
		if (NewSequence == 0) {	// If last step was the 2nd of a sequence start a new one
			NewSequence = 1;
		} else {	// If side is same as last one start a new sequence
			if (LastSide == SIDE_LEFT) NewSequence = 1; else NewSequence = 0;
			// But reverse the roles of the wheels if going backward
			if (Fwd_Rev_Left == FR_REVERSE) NewSequence ^= 1;
		}
		// Remember that this side moved (but switch wheel roles if going backward)
		if (Fwd_Rev_Left == FR_REVERSE) LastSide = SIDE_RIGHT; else LastSide = SIDE_LEFT;

		if (NewSequence == 1) {
			// Update position first
			if (Fwd_Rev_Left != FR_REVERSE) {
				bot_x = bot_x + bot_cos * L_SIN_DELTA;
				bot_y = bot_y + bot_sin * L_SIN_DELTA;
			} else {
				bot_x = bot_x - bot_cos * L_SIN_DELTA;
				bot_y = bot_y - bot_sin * L_SIN_DELTA;
			}
			// Then update angle
			if (LastSide == SIDE_RIGHT) { // SIDE_RIGHT not a typo
				bcos = (bot_cos * COS_DELTA) - (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) + (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			} else {
				bcos = (bot_cos * COS_DELTA) + (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) -(bot_cos * SIN_DELTA);
				bot_cos = bcos;
			}

		} else {
			// Update angle first
			if (LastSide == SIDE_RIGHT) { // SIDE_RIGHT not a typo
				bcos = (bot_cos * COS_DELTA) - (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) + (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			} else {
				bcos = (bot_cos * COS_DELTA) + (bot_sin * SIN_DELTA);
				bot_sin = (bot_sin * COS_DELTA) - (bot_cos * SIN_DELTA);
				bot_cos = bcos;
			}
			// Then update position
			if (Fwd_Rev_Left != FR_REVERSE) {
				bot_x = bot_x + bot_cos * L_SIN_DELTA;
				bot_y = bot_y + bot_sin * L_SIN_DELTA;
			} else {
				bot_x = bot_x - bot_cos * L_SIN_DELTA;
				bot_y = bot_y - bot_sin * L_SIN_DELTA;
			}
		}

	}
}

void set_speeds_raw(int speedLeft, int speedRight)
{
	// speedLeft, speedRight ranges: Backward  Linear  Stop  Linear   Forward
	//                               -200      -100......0......100       200
	servoLeft.writeMicroseconds(L_SERVO_CENTER + speedLeft);   // Set Left servo speed
	servoRight.writeMicroseconds(R_SERVO_CENTER - speedRight); // Set right servo speed

	if (speedLeft>0) Fwd_Rev_Left = 1;
	else if (speedLeft<0) Fwd_Rev_Left = -1;
	else Fwd_Rev_Left = 0;
	
	if (speedRight>0) Fwd_Rev_Right = 1;
	else if (speedRight<0) Fwd_Rev_Right = -1;
	else Fwd_Rev_Right = 0;
}

void set_speeds_calibrated(int speedLeft, int speedRight)
{
	// Add a calibration curve here if necessary
	set_speeds_raw(speedLeft, speedRight);	
}

void align_wheels()
{
	int i, VS, minv;
	
	set_speeds_raw(0, 30);
	minv = 2000;
	for (i=0; i<500; i++) {
		delay(1);
		VS = analogRead(P_Right_Encoder);
		if (VS < minv) minv = VS;
	}
	minv += 5;
	for (i=0; i<2000; i++) {
		delay(1);
		VS = analogRead(P_Right_Encoder);
		if (VS < minv) break;
	}
	set_speeds_raw(0, 0);
	set_speeds_raw(30, 0);
	minv = 2000;
	for (i=0; i<500; i++) {
		delay(1);
		VS = analogRead(P_Left_Encoder);
		if (VS < minv) minv = VS;
	}
	minv += 5;
	for (i=0; i<2000; i++) {
		delay(1);
		VS = analogRead(P_Left_Encoder);
		if (VS < minv) break;
	}
	set_speeds_raw(0, 0);
}

void reset_odometry()
{
	bot_x = 0.0;
	bot_y = 0.0;
	bot_sin = 1.0;	// assume robot is headed in the positive y axis direction
	bot_cos = 0.0;
	NewSequence = 0;
	LastSide = SIDE_RIGHT;
}

void reset_encoder_counts()
{
	ECount_Left = 0;
	ECount_Right = 0;
	WCount_Left = 0;
	WCount_Right = 0;
}

// Goal Seeking PID -------------------------------------------
//
// go_to_goal()
//	Initializes the GO TO controller to seek a new goal
//	Inputs:
//		double gx, gy: The coordinates of the new goal
//		int speed: The wheel speed at which robot should seek the goal
//		double tolerance: If the robot is within this distance from the goal it is assumed the
//			robot has reached its goal.
//	When the robot reaches its goal, the wheel motors are stopped, and go_to_done is set to 1
//   
void go_to_goal(double gx, double gy, int speed, double tolerance)
{
	go_to_done = 0;
	go_to_x = gx;
	go_to_y = gy;
	go_to_speed = speed;
	go_to_old_err = 0.0;
	go_to_tolerance = tolerance;
	Iterm = 0.0;
	if (digitalRead(ZERO_SERVOS) == LOW) {
		set_speeds_calibrated(0, 0);
	} else {
		set_speeds_calibrated(go_to_speed, go_to_speed);
	}
}
//
// go_to_goal_idx()
//	Provides the same functionality as go_to_goal() but the input parameter
//	is used to index a table in flash of struct go_to_st. The inputs to
//	go_to_goal are taken from the table entry.
//
void go_to_goal_idx(int idx)
{
	int i;
	char *ramp;
	char *flashp;
	
	flashp = (char *) &(go_to_table[idx]);
	ramp = (char *) &(go_to_point);
	for (i=0; i<sizeof(struct go_to_st); i++) {
		*ramp = pgm_read_byte_near(flashp);
		ramp++;
		flashp++;
	}
	go_to_goal(go_to_point.gx, go_to_point.gy, go_to_point.speed, go_to_point.tolerance);
}
//
// go_to_goal_PID() performs the GO TO controller function
//	although it is named _PID, the implemented controller can
//	be pure P, PI or PID.
//
void go_to_goal_PID()
{
	double dx, dy;
	int speed;
	
	if (go_to_done) return;
	dx = go_to_x - bot_x;
	dy = go_to_y - bot_y;
	goal_heading = atan2(dy, dx);
	bot_heading = atan2(bot_sin, bot_cos);
	go_to_distance = hypot(dy, dx);
	if (go_to_distance < go_to_tolerance) {
		go_to_done = 1;
		set_speeds_calibrated(0, 0);
		return;
	}
	go_to_err = goal_heading - bot_heading;
	
	if (go_to_err > M_PI)
		go_to_err = M_PI - go_to_err;
	else if (go_to_err < (-(M_PI)))
		go_to_err += 2*M_PI;
	
	speed = go_to_speed;
	
	Pterm = go_to_err * GO_TO_KP;
	
	// To convert this to full PID uncomment the following lines
	if (abs(go_to_PID) > TOO_MUCH_ERROR)
		// Apply Integral Leak-Down
		Iterm = Iterm *= 0.995; // e^(-1/(T * Fs))
	else
		Iterm = Iterm + go_to_err * GO_TO_KI;
	
	Dterm = GO_TO_KP * (go_to_err - go_to_old_err);

	go_to_PID = Pterm + Iterm + Dterm;
	
	right_speed = speed + go_to_PID;
	left_speed = speed - go_to_PID;
	if (right_speed < -110) right_speed = -110;
	if (right_speed > 110) right_speed = 110;
	if (left_speed < -110) left_speed = -110;
	if (left_speed > 110) left_speed = 110;
	go_to_old_err = go_to_err;
	if (digitalRead(ZERO_SERVOS) == LOW) {
		set_speeds_calibrated(0, 0);
	} else {
		set_speeds_calibrated(left_speed, right_speed);
	}
}

void periodicInterrupt() { // Timer2 interrupt calls this function every 50us for 0.83 cm accuracy
	sei();	// Allow other interrupts to get through especially servo timer1
			// However this better be done in less than 50us or else we'll have a big crash
	if (ping_in_progress && sonar.check_timer()) {
		ping_in_progress = 0;
		us_ping_result = sonar.ping_result;
	}
	isr_interrupt_count++;
	// Perform polling tasks below every 10 interrupts or 500 us
	// Stagger tasks to limit real time used during any one interrupt
	
	// Task 1 Check for preamble
	if (isr_interrupt_count == 5) {
		if (*irrxInput & irrxBit) {
			isr_input_pin = 0;	// no carrier
			if (isr_preamble_count == 9) {
				// if preamble count was pegged when
				// carrier goes away then start polling bits
				isr_poll_bits = 1;
				isr_ir_available = 0;
				isr_ir_value = 0;
			}
			isr_preamble_count = 0;
		} else {
			isr_input_pin = 1;	// carrier present
			isr_preamble_count++;
			// Preamble should be at least 5 ms long or 10 x 500 us
			// If carrier has been around that long then peg the
			// preamble counter and clear the bit polling counter
			if (isr_preamble_count > 9) {
				isr_preamble_count = 9;
				isr_poll_bits = 0;
			}
		}
		
	// Task 2 Accumulate bits
	} else if (isr_interrupt_count > 9) {
		isr_interrupt_count = 0;
		if (isr_poll_bits > 0) {
			isr_poll_bits++;
			// Sample bits at 2.0, 5.0, 8.0 and 12.0 ms
			// after end of preamble
			if (isr_poll_bits == 6) {
				if (isr_input_pin) isr_ir_value |= 8;
			} else if (isr_poll_bits == 12) {
				if (isr_input_pin) isr_ir_value |= 4;
			} else if (isr_poll_bits == 18) {
				if (isr_input_pin) isr_ir_value |= 2;
			} else if (isr_poll_bits == 24) {
				if (isr_input_pin) isr_ir_value |= 1;
				isr_ir_available = 1;
			}
			if (isr_poll_bits > 32) isr_poll_bits = 32;
		}
	}
}


