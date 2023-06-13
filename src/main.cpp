#include <GCodeParser.h>
// GCode states.
bool absoluteMode = false;
float zoom = 1.0;
double X_cur=0.0,Y_cur=0.0,X_temp,Y_temp;
GCodeParser GCode = GCodeParser();

#include <AccelStepper.h>
#include <magicFrame.h>

#define DEBUGGER_MODE
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
AccelStepper M1(AccelStepper::DRIVER,M1_STEP_PIN,M1_DIR_PIN);
AccelStepper M2(AccelStepper::DRIVER,M2_STEP_PIN,M2_DIR_PIN);
const u_int16_t STEP_SIZE = AccelStepper::FULL4WIRE;
const float STEPS_PER_REV = STEP_SIZE * 512/3.1415;
const long GEAR_DIA = 16.0; //mm
const u_int16_t WIDTH=550, HEIGHT=600; //mm
bool run_bool=true;

void Steppers_Init(){
  float maxSpeed = 1500.0, maxAcc = 300.0;
  M1.setMaxSpeed(maxSpeed);
  M1.setAcceleration(maxAcc);
  M1.setSpeed(maxSpeed);
  M2.setMaxSpeed(maxSpeed);
  M2.setAcceleration(maxAcc);
  M2.setSpeed(maxSpeed);
  // M1.setPinsInverted(true,false,false);
  // M2.setPinsInverted(true,false,false);
}

void setup()
{
	Steppers_Init();
	pinMode(X_LIMIT_PIN,INPUT_PULLUP);
	pinMode(Y_LIMIT_PIN,INPUT_PULLUP);
	attachInterrupt(X_LIMIT_PIN,limitStop,FALLING);
	attachInterrupt(Y_LIMIT_PIN,limitStop,FALLING);
  	pinMode(LED1,OUTPUT); 
	Serial.begin(115200);
	delay(10);
	Serial.println("Begin");
	// calibrationToOrigin();
}

void loop()
{
  	digitalWrite(LED1,LOW);
	while (Serial.available() > 0){
    if (GCode.AddCharToLine(Serial.read())){
		digitalWrite(LED1,HIGH);
		GCode.ParseLine();
		processCommand();
    }
  }
}

inline double clamp(double value, double minValue, double maxValue){
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

void limitStop(){
	M1.stop();M2.stop();
	run_bool=false;
	#ifdef DEBUGGER_MODE
		if(!digitalRead(X_LIMIT_PIN))
			Serial.println("X Limit Switch");
		else
			Serial.println("Y Limit Switch");
	#endif
}

void calibrationToOrigin(){
	coreXYMove(-1.1*WIDTH,0);
	X_cur=0;
	coreXYMove(0,-1.1*HEIGHT);
	Y_cur=0;
	return;
}

void coreXYMoveTo(double X_des, double Y_des){
	coreXYMove(X_des-X_cur,Y_des-Y_cur);
}

void coreXYMove(double dx, double dy){
	long dM1 = dx+dy, dM2 = dx-dy;
	#ifdef DEBUGGER_MODE
		Serial.print("dM1: ");
		Serial.print(dM1);
		Serial.print("  dM2: ");
		Serial.println(dM2);
	#endif
	M1.move(dM1*STEPS_PER_REV/GEAR_DIA);
	M2.move(dM2*STEPS_PER_REV/GEAR_DIA);
	while(M1.run() || M2.run()){
		if (!run_bool)
			break;
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
    X_temp = X_cur; Y_temp = Y_cur;
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
		X_temp = clamp(X_temp,0,WIDTH);
		Y_temp = clamp(Y_temp,0,HEIGHT);
		
		Serial.println("testing");
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

			coreXYMoveTo(X_temp,X_temp);
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
			coreXYMoveTo(X_temp,X_temp);
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
			logger(ERROR,"Circular Movement not configured");
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
    // Serial.println("M-Code");
    int mCodeNumber = (int)GCode.GetWordValue('M');
    switch (mCodeNumber){
		// M0 - Program Stop
		case 0:
			M1.stop();
			M1.stop();
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
			Serial.print("x: ");
			Serial.print(X_cur);
			Serial.print(" y: ");
			Serial.println(Y_cur);
			Serial.print("Absolute Mode: ");
			Serial.println(absoluteMode);
			break;
      }
    }
  }
}


