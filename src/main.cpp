#include <GCodeParser.h>
// GCode states.
bool absoluteMode = false;
double zoom = 1.0;
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

#define motorPin1a  2    
#define motorPin2a  3    
#define motorPin3a  4    
#define motorPin4a  5    

#define motorPin1b  6    
#define motorPin2b  7    
#define motorPin3b  8    
#define motorPin4b  9    

#define motorPin1c  21
#define motorPin2c  20
#define motorPin3c  19
#define motorPin4c  18

#define motorPin1d  10
#define motorPin2d  11
#define motorPin3d  12
#define motorPin4d  13

/*
  COORD SYSTEM
   r1      r3
     \    /
      \  /
       ()
      /  \
     /    \
   r2      r4
   |->x
 s1-s2----s4-s3
With the (0,0) origin at the point above s2
*/

// void processCommand(bool noSerialResponse = false


// const int STEP_SIZE = AccelStepper::FULL4WIRE;
const u_int16_t STEP_SIZE = AccelStepper::HALF4WIRE;
const float STEPS_PER_REV = STEP_SIZE * 512/3.1415;
const u_int16_t WIDTH = 250; //mm from s2 to s3
const u_int16_t HEIGHT = 488; //mm from  s2 to the start of r1
const float MAX_MOVE = .5; //mm; largest partial move to make between two points
const u_int16_t PULLEY_DIA = 19;//mm; diameter of pulley wheel
#define MAX_MESSAGE_LENGTH 16
#define MIN_X 10
#define MAX_X 240
#define MIN_Y 10
#define MAX_Y 440

// NOTE: The sequence 1-3-2-4 is required for proper sequencing of 28BYJ-48
AccelStepper stepper1(STEP_SIZE, motorPin1a, motorPin3a, motorPin2a, motorPin4a,true);
AccelStepper stepper2(STEP_SIZE, motorPin1b, motorPin3b, motorPin2b, motorPin4b,true);
AccelStepper stepper3(STEP_SIZE, motorPin1c, motorPin3c, motorPin2c, motorPin4c,true);
AccelStepper stepper4(STEP_SIZE, motorPin1d, motorPin3d, motorPin2d, motorPin4d,true);
float x_0,y_0,x_i,y_i,r1,r2,r3,r4;

char msg[32];

static char message[MAX_MESSAGE_LENGTH];
static unsigned int message_pos = 0;


void initializeSteppers(){
  float maxSpeed = 1500.0, maxAcc = 300.0;
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAcc);
  stepper1.setSpeed(maxSpeed);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAcc);
  stepper2.setSpeed(maxSpeed);
  stepper3.setMaxSpeed(maxSpeed);
  stepper3.setAcceleration(maxAcc);
  stepper3.setSpeed(maxSpeed);
  stepper4.setMaxSpeed(maxSpeed);
  stepper4.setAcceleration(maxAcc);
  stepper4.setSpeed(maxSpeed);
  // stepper1.setPinsInverted(true,false,false);
  // stepper2.setPinsInverted(true,false,false);
  // stepper3.setPinsInverted(true,false,false);
  // stepper4.setPinsInverted(true,false,false);
}


void calibrateGlobals(char mode, float AX, float BY){
  if(mode=='C'){
    x_i=AX;
    y_i=BY;
    r1=mag(AX,HEIGHT-BY);
    r2=mag(AX,BY);
    r3=mag(WIDTH-AX,HEIGHT-BY);
    r4=mag(WIDTH-AX,BY);
    stepper1.setCurrentPosition(r1*STEPS_PER_REV/PULLEY_DIA);
    stepper2.setCurrentPosition(r2*STEPS_PER_REV/PULLEY_DIA);
    stepper3.setCurrentPosition(r3*STEPS_PER_REV/PULLEY_DIA);
    stepper4.setCurrentPosition(r4*STEPS_PER_REV/PULLEY_DIA);
  }
  else if(mode=='R'){
    r2=AX;
    r4=BY;
    x_i=(pow(r2,2)-pow(r4,2)+pow(WIDTH,2))/(2*WIDTH);
    y_i=sqrt(pow(r2,2)-pow(x_i,2));
    r1=mag(x_i,HEIGHT-y_i);
    r3=mag(WIDTH-x_i,HEIGHT-y_i);
    stepper1.setCurrentPosition(r1*STEPS_PER_REV/PULLEY_DIA);
    stepper2.setCurrentPosition(r2*STEPS_PER_REV/PULLEY_DIA);
    stepper3.setCurrentPosition(r3*STEPS_PER_REV/PULLEY_DIA);
    stepper4.setCurrentPosition(r4*STEPS_PER_REV/PULLEY_DIA);
  }
  else{
    Serial.println("CALI ERROR");
  }
  
}

void updateGlobals(){
  r1=stepper1.currentPosition()*PULLEY_DIA/STEPS_PER_REV;
  r2=stepper2.currentPosition()*PULLEY_DIA/STEPS_PER_REV;
  r3=stepper3.currentPosition()*PULLEY_DIA/STEPS_PER_REV;
  r4=stepper4.currentPosition()*PULLEY_DIA/STEPS_PER_REV;
  x_i=(WIDTH*WIDTH+r2*r2-(r4*r4))/(2*WIDTH);
  y_i=(HEIGHT*HEIGHT+r2*r2-(r1*r1))/(2*HEIGHT);
  float tol = .1;
  if(abs(mag(x_i,y_i)-r2)>tol){
    #ifdef DEBUGGER_MODE
    logger(ERROR,"Global variables off");
    #endif
  }
}

void moveR(float R1, float R2, float R3, float R4){
  stepper1.moveTo(R1*STEPS_PER_REV/PULLEY_DIA);
  stepper2.moveTo(R2*STEPS_PER_REV/PULLEY_DIA);
  stepper3.moveTo(R3*STEPS_PER_REV/PULLEY_DIA);
  stepper4.moveTo(R4*STEPS_PER_REV/PULLEY_DIA);
  
  while (stepper1.isRunning()||stepper2.isRunning()||stepper3.isRunning()||stepper4.isRunning()){
    // sprintf(msg,"d1:%ld d2:%ld d3:%ld d4:%ld",stepper1.distanceToGo(),stepper2.distanceToGo(),stepper3.distanceToGo(),stepper4.distanceToGo());
    // Serial.println(msg);
    stepper1.run();
    stepper4.run();
    stepper3.run();
    stepper2.run();
    delayMicroseconds(1);
  }
  
}

void moveR_paired(float R1, float R2, float R3, float R4){
  stepper1.moveTo(R1*STEPS_PER_REV/PULLEY_DIA);
  stepper2.moveTo(R2*STEPS_PER_REV/PULLEY_DIA);
  stepper3.moveTo(R3*STEPS_PER_REV/PULLEY_DIA);
  stepper4.moveTo(R4*STEPS_PER_REV/PULLEY_DIA);
  
  while (stepper2.isRunning()||stepper4.isRunning()){
    stepper4.run(); stepper2.run();
  }

  while (stepper1.isRunning()||stepper3.isRunning()){
    stepper1.run(); stepper3.run();
  }
  
}

float mag(float vector0, float vector1){
  return sqrt(vector0*vector0+vector1*vector1);
}

float distTo(float X,float Y){
  return sqrt(abs(X-x_i)*abs(X-x_i)+abs(Y-y_i)*abs(Y-y_i));
}

void move(float X,float Y){
  updateGlobals();
  // sprintf(msg,"x_i:%f  y_i:%f X:%f Y:%f/n",x_i,y_i,X,Y);
  // Serial.println(msg);
  if (distTo(X,Y) > MAX_MOVE){
    move((x_i+X)/2,(y_i+Y)/2);
    move(X,Y);
    return;
  }
  moveR(mag(X,HEIGHT-Y),mag(X,Y),mag(WIDTH-X,HEIGHT-Y),mag(WIDTH-X,Y));
}

void move_paired(float X,float Y){
  updateGlobals();
  // sprintf(msg,"x_i:%f  y_i:%f X:%f Y:%f/n",x_i,y_i,X,Y);
  // Serial.println(msg);
  if (distTo(X,Y) > MAX_MOVE){
    move((x_i+X)/2,(y_i+Y)/2);
    move(X,Y);
    return;
  }
  moveR_paired(mag(X,HEIGHT-Y),mag(X,Y),mag(WIDTH-X,HEIGHT-Y),mag(WIDTH-X,Y));
}

void setup(){
  // Serial.begin(9600);
  initializeSteppers();
  Serial.begin(115200);
  // calibrateGlobals('C',133.35,266.7); //rough paper coords
  calibrateGlobals('C',133.35+16.2,266.7); //calibration for paper coords
  Serial.println("Ready");
}



void loop(){
  // Serial.println("Start");
  // move(x_i+10,y_i+10);
  // Serial.println("done");
  // delay(1000);

  while (Serial.available() > 0){
    if (GCode.AddCharToLine(Serial.read())){
      GCode.ParseLine();
      processCommand();
    }
  }
}

inline double clamp(double value, double minValue, double maxValue){
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

void processCommand(bool noSerialResponse){
  if (GCode.blockDelete)
    return;
  
  if (GCode.HasWord('G')){
    int gCodeNumber = (int)GCode.GetWordValue('G');
    double tempX = x_i;
    double tempY = y_i;
    if (gCodeNumber >= 0 && gCodeNumber <= 3){ // G0, G1, G2, G3
      if (GCode.HasWord('X')){
          if (absoluteMode)
            tempX = GCode.GetWordValue('X') * zoom;
          else
            tempX += GCode.GetWordValue('X') * zoom;
      }
      if (GCode.HasWord('Y')){
          if (absoluteMode)
            tempY = GCode.GetWordValue('Y') * zoom;
          else
            tempY += GCode.GetWordValue('Y') * zoom;
      }
      Serial.println(tempX);
      tempX = clamp(tempX,MIN_X,MAX_X);
      tempY = clamp(tempY,MIN_Y,MAX_Y);
      Serial.println(tempX);
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

        move_paired(tempX,tempY);
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
        move(tempX,tempY);
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
    updateGlobals();
  }


  else if (GCode.HasWord('M')){ // M-Codes
    Serial.println("M-Code");
    int mCodeNumber = (int)GCode.GetWordValue('M');
    switch (mCodeNumber){
      // M099 - sets current coords
      case 99:{
        float tempX=-1,tempY=-1;
        if (GCode.HasWord('C')){
          if(GCode.HasWord('X') && GCode.HasWord('Y')){
            tempX = GCode.GetWordValue('X');
            tempY = GCode.GetWordValue('Y');
            calibrateGlobals('C',tempX,tempY);
          }
          break;
        }
        else if(GCode.HasWord('R')){
          if(GCode.HasWord('A') && GCode.HasWord('B') ){
            tempX = GCode.GetWordValue('A');
            tempY = GCode.GetWordValue('B');
            calibrateGlobals('R',tempX,tempY);// r2 and r4
          }
          break;
        }
      }
      // M999 - Prints Global Values
      case 999:{
        Serial.print("x: ");
        Serial.print(x_i);
        Serial.print(" y: ");
        Serial.println(y_i);
        Serial.print("R: ");
        Serial.print(r1);
        Serial.print(", ");
        Serial.print(r2);
        Serial.print(", ");
        Serial.print(r3);
        Serial.print(", ");
        Serial.println(r4);
        Serial.print("Absolute Mode: ");
        Serial.println(absoluteMode);
        break;
      }
    }
  }
}
