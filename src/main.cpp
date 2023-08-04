#include <Arduino.h>
#include <GCodeParser.h>
#include "RP2040_ISR_Servo.h"

// SERVO Definition 
#if ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)
  #if !defined(RP2040_ISR_SERVO_USING_MBED)    
    #define RP2040_ISR_SERVO_USING_MBED     false
  #endif  
  
#elif ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
      
  #if !defined(RP2040_ISR_SERVO_USING_MBED)    
    #define RP2040_ISR_SERVO_USING_MBED     true
  #endif  
  
#else      
  #error This code is intended to run on the mbed / non-mbed RP2040 platform! Please check your Tools->Board setting.
#endif

// GCode states.
bool absoluteMode = false;
float zoom = 1.0;
double X_cur=0.0,Y_cur=0.0,X_temp,Y_temp;
bool Z_up,Z_temp;
int pen_servo_index;
GCodeParser GCode = GCodeParser();

#include <AccelStepper.h>
#include <magicFrame.h>

// #define DEBUGGER_MODE
#ifdef DEBUGGER_MODE

#define ERROR 11
#define LOG 12
#define WARNING 13


void logger(int TYPE,char const msg[]){
  switch (TYPE)
  {
  case ERROR:
    Serial.print("ERROR: ");
    break;
  case WARNING:
    Serial.print("WARNING: ");
    break;
  case LOG:
    Serial.print("LOG: ");
    break;
  }
  Serial.println(msg);
}

#endif

const int X_LIMIT_PIN = 6, Y_LIMIT_PIN = 7;
const int M1_DIR_PIN = 2, M1_STEP_PIN = 3;
const int M2_DIR_PIN = 4, M2_STEP_PIN = 5;
const int SERVO_PIN = 22;
const int SERVO_POS_DOWN = 165, SERVO_POS_UP = 90;
AccelStepper M1(AccelStepper::DRIVER,M1_STEP_PIN,M1_DIR_PIN);
AccelStepper M2(AccelStepper::DRIVER,M2_STEP_PIN,M2_DIR_PIN);
// const u_int16_t STEP_SIZE = AccelStepper::FULL4WIRE;
const u_int16_t STEP_SIZE = AccelStepper::HALF4WIRE;
// const u_int16_t STEP_SIZE = 2; //quarter step MAY NOT WORK
const float STEPS_PER_REV = STEP_SIZE/4.0 * 200*.3;
const float GEAR_DIA = 12.22; //mm
const u_int16_t WIDTH=550, HEIGHT=600; //mm
bool run_bool=true;
volatile bool x_limit=false, y_limit=false;

void Steppers_Init(){
  float maxSpeed = 200*STEP_SIZE/4.0, maxAcc = 500.0*STEP_SIZE/4.0;
  M1.setMaxSpeed(maxSpeed);
  M1.setAcceleration(maxAcc);
//   M1.setSpeed(maxSpeed);
  M2.setMaxSpeed(maxSpeed);
  M2.setAcceleration(maxAcc);
//   M2.setSpeed(maxSpeed);
  M1.setPinsInverted(true,false,false);
  M2.setPinsInverted(true,false,false);
}

void Servo_Init(){
	pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);

	
	#if defined(ARDUINO_ARCH_MBED)
	Serial.print(F("\nStarting RP2040_MultipleServos on Mbed "));
	#else
	Serial.print(F("\nStarting RP2040_MultipleServos on "));
	#endif

	Serial.println(BOARD_NAME);
	Serial.println(RP2040_ISR_SERVO_VERSION);
	pen_servo_index=RP2040_ISR_Servos.setupServo(SERVO_PIN,80,2450);
	if (pen_servo_index != -1){
		Serial.println("Servo Setup Success");
		RP2040_ISR_Servos.setPosition(pen_servo_index,0);
	}
	else{
		Serial.println("Setup Fail");
		while(1){delay(1);}
	}
	Z_up=false;
	penMoveUp(true);
}

void setup()
{	Serial.begin(115200);
delay(5000);
	Steppers_Init();
	Servo_Init();

	attachInterrupt(digitalPinToInterrupt(X_LIMIT_PIN),limitStop_X,RISING);
	attachInterrupt(digitalPinToInterrupt(Y_LIMIT_PIN),limitStop_Y,RISING);
	pinMode(X_LIMIT_PIN,INPUT_PULLDOWN);
	pinMode(Y_LIMIT_PIN,INPUT_PULLDOWN);

  	pinMode(LED1,OUTPUT); 
	
	delay(10);
	Serial.println("Begin");
	// calibrationToOrigin();
}

void loop()
{
  	digitalWrite(LED1,LOW);
	updateValues(); //may not need to cal this frequent
	while (Serial.available() > 0){
		if (GCode.AddCharToLine(Serial.read())){
			digitalWrite(LED1,HIGH);
			GCode.ParseLine();
			processCommand();
			Serial.println(".");
    	}
	}
	#ifdef DEBUGGER_MODE
	if(x_limit){Serial.println("X Limit Switch");x_limit=false;}
	if(y_limit){Serial.println("Y Limit Switch");y_limit=false;}
	#endif
	// TODO: Clean this up
	delay(10);
}

inline double clamp(double value, double minValue, double maxValue){
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

void penMoveUp(bool true_if_up){
	if (Z_up==true_if_up)
		return;

	if (true_if_up){
		RP2040_ISR_Servos.setPosition(pen_servo_index, SERVO_POS_UP);
	}
	else{
		RP2040_ISR_Servos.setPosition(pen_servo_index, SERVO_POS_DOWN);
	}
	Z_up=true_if_up;
	delay(250);
}

void limitStop_X(){
	if(!digitalRead(X_LIMIT_PIN))
		return;
	run_bool=false;x_limit=true;
	return;
}

void limitStop_Y(){
	if(!digitalRead(Y_LIMIT_PIN))
		return;
	run_bool=false;y_limit=true;
	return;
}

void calibrationToOrigin(){
	int speed_=100*STEP_SIZE/4.0,home_pos=40;
	x_limit=false;y_limit=false;run_bool=true;
	M1.move(-1*WIDTH*STEPS_PER_REV/GEAR_DIA);M2.move(-1*HEIGHT*STEPS_PER_REV/GEAR_DIA);
	M1.setSpeed(speed_);M2.setSpeed(speed_);
	while(!x_limit){M1.run();M2.run();}
	M1.move(-1*WIDTH*STEPS_PER_REV/GEAR_DIA);M2.move(HEIGHT*STEPS_PER_REV/GEAR_DIA);
	M1.setSpeed(speed_);M2.setSpeed(speed_);
	while(!y_limit){M1.run();M2.run();}
	X_cur=0;Y_cur=0;
	M1.setCurrentPosition(0);M2.setCurrentPosition(0);
	M1.move(2*home_pos*STEPS_PER_REV/GEAR_DIA);
	while (M1.isRunning()){M1.run();}
	x_limit=false;y_limit=false;run_bool=true;
	#ifdef DEBUGGER_MODE
	Serial.print("Calibration Done");
	#endif
	return;
}

void updateValues(){
	double M1_mm, M2_mm;
	M1_mm = M1.currentPosition()*GEAR_DIA/STEPS_PER_REV;
	M2_mm = M2.currentPosition()*GEAR_DIA/STEPS_PER_REV;
	X_cur = 0.5*(M1_mm+M2_mm);
	Y_cur = 0.5*(M1_mm-M2_mm);
	return;
}

void coreXYMoveTo(double X_des, double Y_des){
	coreXYMove(X_des-X_cur,Y_des-Y_cur);
	return;
}

void coreXYMove(double dx, double dy){
	long dM1,dM2;
	dM1 = (dx+dy); dM2 = (dx-dy);
	#ifdef DEBUGGER_MODE
		Serial.print("dM1: ");
		Serial.print(dM1);
		Serial.print("  dM2: ");
		Serial.print(dM2);
	#endif
	M1.move(dM1*STEPS_PER_REV/GEAR_DIA);
	M2.move(dM2*STEPS_PER_REV/GEAR_DIA);
	// if(M1.distanceToGo()==0 && M2.distanceToGo()==0)
	// 	return;
	while(M1.isRunning() || M2.isRunning()){
		if (!run_bool){
			M1.stop();M2.stop();
			while (M1.runSpeedToPosition() || M1.runSpeedToPosition()) {}
			M1.move(0);M2.move(0);
			break;
		}
		M1.run();M2.run();
		delayMicroseconds(1);
		//some updating display could go here
	}
	return;
}

void processCommand(bool noSerialResponse){
  if (GCode.blockDelete)
    return;
  
  if (GCode.HasWord('G')){
    int gCodeNumber = (int)GCode.GetWordValue('G');
    X_temp = X_cur; Y_temp = Y_cur; Z_temp = Z_up;
    if (gCodeNumber >= 0 && gCodeNumber <= 3){ // G0, G1, G2, G3
		if (GCode.HasWord('X')){
			if (absoluteMode)
				X_temp = GCode.GetWordValue('X') * zoom;
			else
				X_temp += GCode.GetWordValue('X') * zoom;
		}
		if (GCode.HasWord('Y')){
			if (absoluteMode)
				Y_temp = GCode.GetWordValue('Y') * zoom;
			else
				Y_temp += GCode.GetWordValue('Y') * zoom;
		}
		if (GCode.HasWord('Z')){
			Z_temp = GCode.GetWordValue('Z') > 0;
		}
		// Serial.print(X_temp);Serial.print("  ");Serial.println(Y_temp);
		// X_temp = clamp(X_temp,0,WIDTH);
		// Y_temp = clamp(Y_temp,0,HEIGHT);
    }

    switch (gCodeNumber){
    	// G00 – Rapid Positioning
		// The G00 command moves the machine at maximum travel speed from a current
		// position to a specified point or the coordinates specified by the command.
		// The machine will move all axis at the same time, so they complete the
		// travel simultaneously. This results in a straight-line movement to the new
		// position point.
		case 0:
			// moves r2 and r4 first then r1 and r3 in hopes the tensioners provide a buffer and stop the motors from working against each other
			penMoveUp(Z_temp);
			coreXYMoveTo(X_temp,Y_temp);
			break;

		// G01 – Linear Interpolation
		// The G01 G-code command instructs the machine to move in a straight line
		// at a set feed rate or speed. We specify the end position with the X, Y
		// and Z values, and the speed with the F value. The machine controller
		// calculates (interpolates) the intermediate points to pass through to get
		// that straight line. Although these G-code commands are simple and quite
		// intuitive to understand, behind them, the machine controller performs
		// thousands of calculations per second in order to make these movements.
		case 1:
			penMoveUp(Z_temp);
			coreXYMoveTo(X_temp,Y_temp);
			break;

		// G02 – Circular Interpolation Clockwise
		// The G02 command tells the machine to move clockwise in a circular pattern.
		// It is the same concept as the G01 command and it’s used when performing
		// the appropriate machining process. In addition to the end point parameters,
		// here we also need to define the center of rotation, or the distance of
		// the arc start point from the center point of the arc. The start point is
		// actually the end point from the previous command or the current point.
		case 2:
		// G03 – Circular Interpolation Counterclockwise
		// Just like the G02, the G03 G-code command defines the machine to move in
		// circular pattern. The only difference here is that the motion is
		// counterclockwise. All other features and rules are the same as the G02
		// command.
		case 3:
			#ifdef DEBUGGER_MODE
			logger(ERROR,"Circular Movement not configured");
			#endif
			break;

		// G04 – Dwell Command
		// G04 is called the Dwell command because it makes the machine stop what it
		// is doing or dwell for a specified length of time. It is helpful to be able
		// to dwell during a cutting operation, and also to facilitate various
		// non-cutting operations of the machine.
		case 4: // G4 - Delay P milliseconds.
			if (GCode.HasWord('P'))
			delay(GCode.GetWordValue('P'));
			break;

		case 28: // G28 - Auto Home
			calibrationToOrigin();
			break;

		// G90/G91 – Positioning G-code commands
		// With the G90 and G91 commands we tell the machine how to interpret the
		// coordinates. G90 is for absolute mode and G91 is for relative mode.
		//
		// In absolute mode the positioning of the tool is always from the absolute
		// point or zero. So, the command G01 X10 Y5 will take the tool to that
		// exact point (10,5), no matter the previous position.
		//
		// On the other hand, in relative mode, the positioning of the tool is
		// relative to the last point. So, if the machine is currently at point
		// (10,10), the command G01 X10 Y5 will take the tool to point (20,15).
		// This mode is also called “incremental mode”.
		case 90: // G90 - Absolute Positioning.
			absoluteMode = true;
			break;
		case 91: // G91 - Incremental Positioning.
			absoluteMode = false;
			break;
    }
    // updateGlobals();
  }

  else if (GCode.HasWord('M')){ // M-Codes
    int mCodeNumber = (int)GCode.GetWordValue('M');
    switch (mCodeNumber){
		// M0 - Program Stop
		case 0:
			M1.stop();
			M1.stop();
			run_bool = false;
			break;
		// M1 - TEST TRIGGER
		#ifdef DEBUGGER_MODE
		case 1:
			Serial.println(digitalRead(Y_LIMIT_PIN));
			break;
		#endif
		// M024 - Resume Print
		case 24:
			run_bool = true;
			M1.enableOutputs();M2.enableOutputs();
			Serial.print("M1 to go: ");Serial.print(M1.distanceToGo());
			Serial.print("  M2 to go: ");Serial.print(M2.distanceToGo());
			break;

		// M099 - sets current coords
		case 99:{
			if(GCode.HasWord('X') && GCode.HasWord('Y')){
				X_cur = GCode.GetWordValue('X');
				Y_cur = GCode.GetWordValue('Y');
			}
			break;
		}
		// M999 - Prints Global Values
		case 999:{
			Serial.print("M1: ");Serial.print(M1.currentPosition());
			Serial.print("  M2: ");Serial.print(M2.currentPosition());
			Serial.print("  x: ");Serial.print(X_cur);
			Serial.print("  y: ");Serial.print(Y_cur);
			Serial.print("  Absolute Mode: ");
			Serial.print(absoluteMode);
			break;
      }
    }
  }
}


