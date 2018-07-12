// Notes: this file contains different experiments to compute acceleration:
//         - use the AccelStepper method, (see http://www.airspayce.com/mikem/arduino/AccelStepper/ http://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf)
//         - pre-compute the braking distance
// but those methods cannot work in a context where the speed of the two motors is not at all linearly related with the speed of the pen

#ifdef SIMULATOR
#include <FakeArduino/Servo.h>
#include <FakeArduino/AccelStepper.h>
using namespace std;
#else
#include <Servo.h>
#include <AccelStepper.h>
#endif

// #define DEBUG

#include "debug.h"

// Constants

const unsigned int STEP_L = 36;
const unsigned int DIRECTION_L = 34;

const unsigned int STEP_R = 60;
const unsigned int DIRECTION_R = 61;

const unsigned int M0_L = A0;
const unsigned int M1_L = A1;
const unsigned int M2_L = A2;

const unsigned int M0_R = A3;
const unsigned int M1_R = A4;
const unsigned int M2_R = A5;

const unsigned int ENABLE_L = 30;
const unsigned int ENABLE_R = 56;

const bool ENABLE_L_INVERTED = true;
const bool ENABLE_R_INVERTED = true;

const unsigned int SERVO = 4;

const unsigned int MIN_MOTOR_PULSE_WIDTH = 1;

// Speed is in steps per seconds (step/s)
// Acceleration in steps per seconds^2 (step/s^2)
const float MAX_MOTOR_SPEED = 5 * 200.0 * 32;               // 32 000 = 5 turns per seconds


// Commands

const unsigned int NUM_PARAMETERS = 8;
float parameters[NUM_PARAMETERS];
bool parametersSet[NUM_PARAMETERS];

const unsigned int PARAMETER_X = 0;
const unsigned int PARAMETER_Y = 1;
const unsigned int PARAMETER_R = 2;
const unsigned int PARAMETER_S = 3;
const unsigned int PARAMETER_G = 4;
const unsigned int PARAMETER_M = 5;
const unsigned int PARAMETER_P = 6;
const unsigned int PARAMETER_F = 7;

int parameterIndex = -1;

String serialInput = "";

typedef enum { COMMAND, PARAMETER, VALUE } ReadingType;

ReadingType readingType = COMMAND;

typedef enum {  IDLE, MOVE_DIRECT, MOVE_DIRECT_MAX_SPEED, MOVE_LINEAR, MOVE_L, MOVE_R, MOVE_PEN, WAIT, SET_POSITION, SET_SPEED, SET_ACCELERATION, ENABLE_MOTORS, DISABLE_MOTORS, SETUP, INVERT_XY, SET_PROGRESSIVE_MICROSTEPS, RESET } Command;
Command command = IDLE;
Command nextCommand = IDLE;

// Speed timings
unsigned long lastSpeedUpdate = 0;
unsigned long lastMotorUpdate = 0;

// Periods
unsigned long nStepsToDoL = 0;
unsigned long nStepsToDoR = 0;
unsigned long nStepsDone = 0;

// Linear steps
unsigned long nLinearStepsToDo = 0;
unsigned long nLinearStepsDone = 0;

// Motor directions
bool clockwiseL = true;
bool clockwiseR = true;

// Settings
float machineWidth = 2000;
float stepsPerRevolution = 200;
unsigned int microstepResolution = 32;
float millimetersPerRevolution = 96;
float millimetersPerStep = millimetersPerRevolution / (stepsPerRevolution * microstepResolution);
bool progressiveMicrosteps = false;
float linearStepDistance = 1.0;               // in millimeter
bool invertX = false;
bool invertY = false;

// Speed and acceleration in steps per seconds
float maxSpeed = 200 * 32;
float minSpeed = 10;
float speed = minSpeed;
float acceleration = 500;

float brakingDistance = 10.0;
float accelerationDuration = 0.0;

// Current state

// Current position and target
float positionX = -1.0;
float positionY = -1.0;
long positionL = 0;
long positionR = 0;
float startX = 0.0;
float startY = 0.0;
float targetX = 0.0;
float targetY = 0.0;
long targetL = 0L;
long targetR = 0L;
long subTargetL = 0L;
long subTargetR = 0L;
unsigned int microstepResolutionL = microstepResolution;
unsigned int microstepResolutionR = microstepResolution;
// Substeps
unsigned int nSubStepsL = 0;
unsigned int nSubStepsR = 0;

//Speed
long speedN = 0;
float speedC0 = 0.0;
float speedCmin = 1.0;
float speedCn = 0.0;
bool deceleration = false;

// motors state
bool motorsOn = false;

// Current servo angle
float servoAngle = 0;


Servo servo;
AccelStepper stepper(AccelStepper::DRIVER);



void reset();
void enableMotors();

void setup() {

  Serial.begin(115200);

  pinMode(M0_L, OUTPUT);
  pinMode(M1_L, OUTPUT);
  pinMode(M2_L, OUTPUT);

  pinMode(M0_R, OUTPUT);
  pinMode(M1_R, OUTPUT);
  pinMode(M2_R, OUTPUT);

  pinMode(ENABLE_L, OUTPUT);
  pinMode(ENABLE_R, OUTPUT);

  pinMode(DIRECTION_L, OUTPUT);
  pinMode(DIRECTION_R, OUTPUT);

  pinMode(STEP_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);

  servo.attach(SERVO);

  enableMotors();

  speedCmin = 1000000.0 / (maxSpeed / millimetersPerStep);
  speedC0 = 0.676 * sqrt(2.0 / (acceleration / millimetersPerStep)) * 1000000.0;
      

  Serial.println("Tipibot ready!");
  Serial.println("Initialize");
}


// --------
// Utils
// --------

float stof(String& str) {
  #ifdef SIMULATOR
    return std::stof(serialInput);
  #else
    return str.toFloat();
  #endif
}

void orthoToPolar(float x, float y, long* l, long* r) {
  float x2 = x * x;
  float y2 = y * y;
  float WmX = machineWidth - x;
  float WmX2 = WmX * WmX;
  *l = round(sqrt(x2 + y2) / millimetersPerStep);
  *r = round(sqrt(WmX2 + y2) / millimetersPerStep);
}

void polarToOrtho(long l, long r, float* x, float* y) {
  float lf = l * millimetersPerStep;
  float rf = r * millimetersPerStep;
  float l2 = lf * lf;
  float r2 = rf * rf;
  float w2 = machineWidth * machineWidth;
  float x2 = *x * *x;
  *x = (l2 - r2 + w2) / ( 2.0 * machineWidth );
  *y = sqrt(l2 - x2);
}

void updatePositions(long currentPositionL, long currentPositionR) {
  polarToOrtho(currentPositionL, currentPositionR, &positionX, &positionY);
}


// ------
// Motors
// ------

void step(unsigned int directionPin, unsigned int stepPin, bool clockwise, unsigned int* nSubSteps, unsigned int motorMicrostepResolution)
{
  // Ignore sub steps
  ++*nSubSteps;
  if(*nSubSteps < microstepResolution / motorMicrostepResolution ) {
    return;
  }
  *nSubSteps = 0;

  digitalWrite(directionPin, clockwise ? HIGH : LOW);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepPin, HIGH);
  
  delayMicroseconds(MIN_MOTOR_PULSE_WIDTH);
  
  digitalWrite(stepPin, LOW);
}

// motor L and R must rotate in opposite direction, but this must not be visible by the user 
// (that is why invertX and invertY work in opposite ways)
// but R position is not inverted

// Direction logic table:
// ----------------------------------
// | clockwise | invert | direction |
// ----------------------------------
// |      0    |    1    |   -1     |
// |      0    |    0    |   1      |
// |      1    |    1    |   1      |
// |      1    |    0    |   -1     |
// ----------------------------------
//
// => direction = clockwise != invert ? 1 : -1
// note that right motor runs in opposite direction from left motor

void stepL()
{
  bool clockwise = invertX != clockwiseL;
  step(DIRECTION_L, STEP_L, clockwise, &nSubStepsL, microstepResolutionL);
  positionL += clockwise ? 1 : -1;
}

void stepR()
{
  bool clockwise = invertY != clockwiseR;
  step(DIRECTION_R, STEP_R, !clockwise, &nSubStepsR, microstepResolutionR);
  positionR += clockwise ? 1 : -1;
}

void stepMotor(bool left) {
  if(left) {
    stepL();
  } else {
    stepR();
  }
}

// Step slowest motor or left motor if motors go at the same speed
void stepSlowestMotor() {
  stepMotor(nStepsToDoL <= nStepsToDoR);
}

// Step fastest motor or right motor if motors go at the same speed
void stepFastestMotor() {
  stepMotor(nStepsToDoL > nStepsToDoR);
}

// -------
// Actions
// -------


void enableMotors() {
  digitalWrite(ENABLE_L, ENABLE_L_INVERTED ? LOW : HIGH);
  digitalWrite(ENABLE_R, ENABLE_R_INVERTED ? LOW : HIGH);
  motorsOn = true;
}

void disableMotors() {
  digitalWrite(ENABLE_L, ENABLE_L_INVERTED ? HIGH : LOW);
  digitalWrite(ENABLE_R, ENABLE_R_INVERTED ? HIGH : LOW);
  motorsOn = false;
}

void accelerate(unsigned long deltaTime) {
  speed += acceleration * (deltaTime / 1000000.0);
  // speed += acceleration / speed;
  speed = min(speed, maxSpeed);

  // if(speed >= maxSpeed) {
  //   speed = maxSpeed;
  // }
}

void decelerate(unsigned long deltaTime) {
  // speed -= acceleration * (deltaTime / 1000000.0);
  speed -= acceleration / speed;
  speed = max(speed, minSpeed);
}

void setMicrostepResolutionFromPower(unsigned int microstepResolutionPower, int M0, int M1, int M2, unsigned int* motorMicrostepResolution) {
  unsigned int p1 = microstepResolutionPower & 1U;
  unsigned int p2 = (microstepResolutionPower >> 1) & 1U;
  unsigned int p3 = (microstepResolutionPower >> 2) & 1U;

  digitalWrite(M0, p1);
  digitalWrite(M1, p2);
  digitalWrite(M2, p3);

  *motorMicrostepResolution = pow(2, microstepResolutionPower);
}

void computeMicrostepResolution(float speed, int M0, int M1, int M2, unsigned int* motorMicrostepResolution) {

  // scale the speed between 0 and microstepResolution (32):
  float scaledSpeed = 32 * speed / MAX_MOTOR_SPEED;
  // compute microstepResolution: given 2^(motorMicrostepResolution+1) = scaledSpeed
  unsigned int microstepResolutionPower = max(5 - int(floor(log(scaledSpeed+1)/log(2))), 0);

  setMicrostepResolutionFromPower(microstepResolutionPower, M0, M1, M2, motorMicrostepResolution);
}

void computeProgressiveMicrostep(unsigned long nStepsToDoL, unsigned long nStepsToDoR) {
    float speedL = speed;
    float speedR = speed;

    if (nStepsToDoL <= nStepsToDoR) {
      speedL = speedR * nStepsToDoL / float(nStepsToDoR);
    } else {
      speedR = speedL * nStepsToDoR / float(nStepsToDoL);
    }

    computeMicrostepResolution(speedL, M0_L, M1_L, M2_L, &microstepResolutionL);
    computeMicrostepResolution(speedR, M0_R, M1_R, M2_R, &microstepResolutionR);
}

// Compute number of steps to do for each motor
void computeStepsToDo() {
  long deltaL = subTargetL - positionL;
  long deltaR = subTargetR - positionR;

  clockwiseL = deltaL >= 0;
  clockwiseR = deltaR >= 0;

  nStepsDone = 0;
  nStepsToDoL = abs(deltaL);
  nStepsToDoR = abs(deltaR);

  if (nStepsToDoL == 0 && nStepsToDoR == 0) {
    // Reached position
    Serial.println("Error: trying to set speed while reached position.");
    return;
  }

  if(progressiveMicrosteps) {
    computeProgressiveMicrostep(nStepsToDoL, nStepsToDoR);
  }

  // SerialPrintln4("--- computeStepsToDo: nStepsToDo: l", nStepsToDoL, "r", nStepsToDoR, "clockwise: l", (unsigned long)clockwiseL, "r", (unsigned long)clockwiseR);
  // SerialPrintln4("--- computeStepsToDo: subTarget: l", subTargetL, "r", subTargetR, "position: l", positionL, "r", positionR);
}

// Compute global speed in orthogonal space (x, y - mm) from acceleration
void computeSpeed(unsigned long deltaTime) {

  // float deltaX1 = abs(positionX - startX);
  // float deltaY1 = abs(positionY - startY);
  // float deltaX2 = abs(positionX - targetX);
  // float deltaY2 = abs(positionY - targetY);

  // accelerationDuration += (deltaTime / 1000000.0);

  // if( ((deltaX1 >= deltaX2 && deltaY1 >= deltaY2) || speed >= maxSpeed-0.001) && brakingDistance < 0.0) {  


  //   float distance = sqrt(deltaX1 * deltaX1 + deltaY1 * deltaY1);
  //   brakingDistance = accelerationDuration * speed - distance;

  // }



  // simple working version:
  float deltaX = targetX - positionX;
  float deltaY = targetY - positionY;

  float distanceToRun = sqrt(deltaX * deltaX + deltaY * deltaY);
  unsigned long nStepsToDo = floor(distanceToRun / millimetersPerStep);

  // if(nStepsToDo < nStepsToStop) {
  //   if(!deceleration) {
  //     SerialPrintln1("computeSpeed: deltaTime", deltaTime, true);
  //     SerialPrintln(deceleration ? "deceleration" : "acceleration", true);
  //     SerialPrintln1("nStepsToDo", nStepsToDo, true);
  //     SerialPrintln1("nStepsToStop", nStepsToStop, true);
  //     SerialPrintln1("speed", speed);
  //   }
  //   deceleration = true;
  //   decelerate(deltaTime);

  // } else if(speed < maxSpeed && !deceleration) {
    
  //   accelerate(deltaTime);

  // }



  // unsigned long nStepsToDo = max(abs(targetL - positionL), abs(targetR - positionR));

  // float deltaL = targetL - positionL;
  // float deltaR = targetR - positionR;
  // float nStepsToDo = floor(sqrt(deltaL * deltaL + deltaR * deltaR));

  // // unsigned long nStepsToDo = max(abs(targetL - positionL), abs(targetR - positionR));
  float stepsPerSeconds = speed / millimetersPerStep;
  long nStepsToStop = (long)((stepsPerSeconds * stepsPerSeconds) / (2.0 * acceleration / millimetersPerStep));

  if(speedN > 0) {
    if(nStepsToStop >= nStepsToDo) {
      speedN = -nStepsToStop;
    }
  } else if(speedN < 0) {
    if(nStepsToStop < nStepsToDo) {
      speedN = -speedN;
    }
  }

  if(speedN == 0) {
    speedCn = speedC0;
  } else {
    speedCn = speedCn - 2.0 * speedCn / ( 4.0 * speedN + 1);
    speedCn = max(speedCn, speedCmin);
  }

  speedN++;
  speed = ( 1000000.0 / fabs(speedCn) ) * millimetersPerStep;

  SerialPrintln1("nStepsToDo", nStepsToDo, true);
  SerialPrintln1("nStepsToStop", nStepsToStop, true);
  SerialPrintln1("speedN", speedN, true);
  SerialPrintln1("speedCn", speedCn, true);
  SerialPrintln1("speedCmin", speedCmin, true);
  SerialPrintln1("speedC0", speedC0, true);
  SerialPrintln1("speed", speed, true);

  // float timeBeforeStop = speed / acceleration;
  // float brakingDistance = (speed * timeBeforeStop) / 2.0;

  // bool deceleration = distanceToRun <= brakingDistance; // && speed > minSpeed;


  // (sqrSpeed / twoa) < distanceTo)


  // SerialPrintln1("speed", speed);
}

// updates

void updatePen() {
  servo.write(servoAngle);
  command = IDLE;
}

void updateMotors(bool mustComputeSpeed = true) {

  stepper.run();

  unsigned long time = micros();
  unsigned long deltaTime = time - lastMotorUpdate;
  
  float stepsPerSeconds = speed / millimetersPerStep;
  float period = 1000000L / stepsPerSeconds;
  // float period = 1000000L / speed;

  if(deltaTime < period) {
    return;
  }
  SerialPrintln1("period", period, true);

  lastMotorUpdate = time;

  if(mustComputeSpeed) {
    computeSpeed(deltaTime);
  }

  unsigned long nStepsToDo = max(nStepsToDoL, nStepsToDoR);
  unsigned long halfNStepsToDoSlowestMotor = min(nStepsToDoL, nStepsToDoR) / 2;

  // At each iteration step:
  //  - both motors at the beginning and at the end of the path ( (n steps to do on slowest motor) / 2 times on each end)
  //  - fastest motor otherwise

  //
  //                         /|
  //   _____________________/ |
  //  /                       |  n steps to do slowest motor
  // /________________________|
  //
  //            n steps to do fastest motor
  //
  //
  // Note: step both motors if they go at the same speed: 
  //       stepFastestMotor & stepSlowestMotor step different motors if motor speeds are equal

  if(nStepsDone < nStepsToDo) {
    stepFastestMotor();
  }

  if(nStepsDone < halfNStepsToDoSlowestMotor || nStepsDone >= nStepsToDo - halfNStepsToDoSlowestMotor) {
    stepSlowestMotor();
  }

  // SerialPrintln4("--- updateMotors: nStepsToDo", nStepsToDo, "halfNStepsToDoSlowestMotor", halfNStepsToDoSlowestMotor, "nStepsDone", nStepsDone, "deltaTime", deltaTime, true);
  // SerialPrintln2("--- updateMotors: position: l", positionL, "r", positionR, true);

  nStepsDone++;

}

void checkTargetReached() {
  
  updatePositions(positionL, positionR);

  printPositions("updateMotorsDirect");

  if(positionL == subTargetL && positionR == subTargetR) {
    command = IDLE;
  }
}

void updateMotorsDirectMaxSpeed() {
  updateMotors(false);
  checkTargetReached();
}

void updateMotorsDirect() {
  updateMotors();
  checkTargetReached();
}

// Called at each step (of size linearStepDistance, in mm) from updateMotorsLinear, always with the same target (same x and y values)
void updateTarget() {

  float deltaX = targetX - startX;
  float deltaY = targetY - startY;

  unsigned long currentStep = nLinearStepsDone + 1;

  float ratio = currentStep / float(nLinearStepsToDo);

  float subTargetX = currentStep < nLinearStepsToDo ? startX + deltaX * ratio : targetX;
  float subTargetY = currentStep < nLinearStepsToDo ? startY + deltaY * ratio : targetY;

  orthoToPolar(subTargetX, subTargetY, &subTargetL, &subTargetR);
  computeStepsToDo();

  // SerialPrintln4("sub target: l", subTargetL, "r", subTargetR, "position l", positionL, "r", positionR);
  // SerialPrintln4("setTargetLinear: target: x", targetX, "y", targetY, "sub target: l", subTargetL, "r", subTargetR);
}

void updateMotorsLinear() {
  updateMotors();

  updatePositions(positionL, positionR);

  printPositions("updateMotorsLinear");
 
  if (positionL == subTargetL && positionR == subTargetR) {
    nLinearStepsDone++;
    if (nLinearStepsDone == nLinearStepsToDo) {
      command = IDLE;
    } else {
      updateTarget();
    }
  }
}

// Set position

void setPosition(float x, float y) {
  targetX = x;
  targetY = y;
  positionX = x;
  positionY = y;
  orthoToPolar(x, y, &positionL, &positionR);
}

// Set targets

void setTarget(float x, float y) {
  startX = positionX;
  startY = positionY;
  targetX = x;
  targetY = y;
  orthoToPolar(targetX, targetY, &targetL, &targetR);
  subTargetL = targetL;
  subTargetR = targetR;
  enableMotors();
  computeStepsToDo();
  lastMotorUpdate = micros();
  speedN = 0;

  // float deltaX = targetX - startX;
  // float deltaY = targetY - startY;

  // float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  brakingDistance = -1.0; //distance / 2.0;
  accelerationDuration = 0.0;
  deceleration = false;

  stepper.setCurrentPosition(positionL);
  stepper.moveTo(targetL);
}

void setTargetDirect(float x, float y) {
  setTarget(x, y);
  // SerialPrintln4("setTargetDirect: target: x", targetX, "y", targetY, "sub target: l", subTargetL, "r", subTargetR);
}

void setTargetDirectMaxSpeed(float x, float y) {
  speed = maxSpeed;
  setTarget(x, y);
  // SerialPrintln4("setTargetDirectMaxSpeed: target: x", targetX, "y", targetY, "sub target: l", subTargetL, "r", subTargetR);
}

void setTargetLinear(float x, float y) {
  setTarget(x, y);

  float deltaX = targetX - startX;
  float deltaY = targetY - startY;

  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  nLinearStepsToDo = distance / linearStepDistance;
  nLinearStepsDone = 0;
  
  updateTarget();
}


// -------------
// Communication
// -------------

bool commandDone() {
  return command == IDLE;
}

bool nextCommandReadyToStart() {
  return nextCommand != IDLE && readingType == COMMAND;
}

void readCommand() {

  if(!Serial.available()) {
    return;
  }

  int byte = Serial.read();

  if(byte == -1) {
    return;
  }

  if(readingType == COMMAND) {

    if(byte != ' ' && byte != '\n') {
      serialInput += (char)byte;
    } else {
      
      if(nextCommand != IDLE && serialInput != "M0") {
        Serial.println("Machine not ready for new commands");
        return;
      }

      if(serialInput == "G0") {
        nextCommand = MOVE_DIRECT;
      } else if(serialInput == "G1") {
        nextCommand = MOVE_LINEAR;
      } else if(serialInput == "G2") {
        nextCommand = MOVE_DIRECT_MAX_SPEED;
      } else if(serialInput == "G4") {
        nextCommand = WAIT;
      } else if(serialInput == "G92") {
        nextCommand = SET_POSITION;
      } else if(serialInput == "M0") {
        nextCommand = RESET;
        command = IDLE;
      } else if(serialInput == "M4") {
        nextCommand = SETUP;
      } else if(serialInput == "M12") {
        nextCommand = INVERT_XY;
      } else if(serialInput == "M13") {
        nextCommand = SET_PROGRESSIVE_MICROSTEPS;
      } else if(serialInput == "M84") {
        nextCommand = DISABLE_MOTORS;
      } else if(serialInput == "M85") {
        nextCommand = ENABLE_MOTORS;
      } else if(serialInput == "M340") {
        nextCommand = MOVE_PEN;
      } else {
        Serial.println("Error: unknown command: ");
        Serial.println(serialInput);
        nextCommand = IDLE;
      }

      for (unsigned int i = 0 ; i < NUM_PARAMETERS ; i++) {
        parameters[i] = -1.0;
        parametersSet[i] = false;
      }

      serialInput = "";

      if(byte == ' ') {
        readingType = PARAMETER;
      } else {
        readingType = COMMAND;
      }
    }

  } else if(readingType == PARAMETER) {

    if(byte != ' ' && byte != '\n') {

      switch (byte)
      {
        case 'X':
        parameterIndex = 0;
        break;
        case 'Y':
        parameterIndex = 1;
        break;
        case 'R':
        parameterIndex = 2;
        break;
        case 'S':
        parameterIndex = 3;
        break;
        case 'G':
        parameterIndex = 4;
        break;
        case 'M':
        parameterIndex = 5;
        break;
        case 'P':
        parameterIndex = 6;
        break;
        case 'F':
        parameterIndex = 7;
        break;
        case 'H':
        parameterIndex = 0;
        break;
        default:
        Serial.println("Error: unknown parameter type: ");
        Serial.println((char)byte);
        parameterIndex = -1;
        break;
      }

      readingType = VALUE;

    } else {
      Serial.println("Error while reading command parameters: parameter name is longer than one character.");
    }

  } else if(readingType == VALUE) {

    if(byte != ' ' && byte != '\n') {
      serialInput += char(byte);
    } else {
      parameters[parameterIndex] = stof(serialInput);
      parametersSet[parameterIndex] = true;

      parameterIndex = -1;
      serialInput = "";

      if(byte == ' ') {
        readingType = PARAMETER;
      } else {
        readingType = COMMAND;
      }
    }

  }

}



// -------
// Command
// -------


void computeBrakingDistance() {
  float stepDuration = max(0.001, 1.0 / (maxSpeed));
  float simulationSpeed = maxSpeed;
  brakingDistance = 0.0;

  SerialPrintln1("simulationSpeed", simulationSpeed);
  SerialPrintln1("stepDuration", stepDuration);

  while(simulationSpeed > minSpeed) {
    simulationSpeed -= acceleration * stepDuration;
    brakingDistance += simulationSpeed * stepDuration;
    stepDuration = max(0.01, 1.0 / (simulationSpeed));
    
  }
  SerialPrintln1("maxSpeed", maxSpeed);
  SerialPrintln1("acceleration", acceleration);
  SerialPrintln1("minSpeed", minSpeed);
  SerialPrintln1("brakingDistance", brakingDistance);
}

// void computeBrakingDistance() {
//   float stepDuration = 1.0 / (minSpeed / millimetersPerStep);
//   float simulationSpeed = minSpeed;
//   brakingDistance = 0.0;

//   SerialPrintln2("simulationSpeed", simulationSpeed, "brakingDistance", brakingDistance);
//   SerialPrintln1("stepDuration", stepDuration);

//   while(simulationSpeed < maxSpeed) {
//     simulationSpeed += acceleration * stepDuration;
//     brakingDistance += simulationSpeed * stepDuration;
//     stepDuration = 1.0 / (simulationSpeed / millimetersPerStep);
    
//     SerialPrintln2("simulationSpeed", simulationSpeed, "brakingDistance", brakingDistance);
//     SerialPrintln1("stepDuration", stepDuration);
//   }
//   SerialPrintln1("brakingDistance", brakingDistance);
// }

void startCommand(bool sendReady) {
  
  command = nextCommand;
  nextCommand = IDLE;

  // TODO: handle errors in parameters

  if(command == MOVE_DIRECT || command == MOVE_DIRECT_MAX_SPEED || command == MOVE_LINEAR) {

    if(parametersSet[PARAMETER_F]) {
      
      maxSpeed = parameters[PARAMETER_F];

      speedCmin = 1000000.0 / (maxSpeed / millimetersPerStep);
      // computeBrakingDistance();

      SerialPrintln1("SET_SPEED: maxSpeed", maxSpeed);

      SerialPrintln1("maxSpeed", maxSpeed / millimetersPerStep, false);
      SerialPrintln1("speedCmin", speedCmin, false);
      stepper.setMaxSpeed(maxSpeed / millimetersPerStep);
    }

    if(parametersSet[PARAMETER_S]) {
      float previousAcceleration = acceleration;
      
      acceleration = parameters[PARAMETER_S];

      speedC0 = 0.676 * sqrt(2.0 / (acceleration / millimetersPerStep)) * 1000000.0;
      // computeBrakingDistance();
      SerialPrintln1("SET_ACCELERATION: acceleration", acceleration);

      SerialPrintln1("acceleration", acceleration / millimetersPerStep, false);
      SerialPrintln1("speedC0", speedC0, false);
      stepper.setAcceleration(acceleration / millimetersPerStep);

    }

    if((parametersSet[PARAMETER_X] || parametersSet[PARAMETER_Y]) && positionX >= 0.0 && positionY >= 0.0) {

      float targetX = parametersSet[PARAMETER_X] ? parameters[PARAMETER_X] : positionX;
      float targetY = parametersSet[PARAMETER_Y] ? parameters[PARAMETER_Y] : positionY;
      
      lastSpeedUpdate = micros();

      if(command == MOVE_DIRECT) {
        setTargetDirect(targetX, targetY);
      } else if(command == MOVE_DIRECT_MAX_SPEED) {
        setTargetDirectMaxSpeed(targetX, targetY);
      } else if(command == MOVE_LINEAR) {
        setTargetLinear(targetX, targetY);
      }
      
    } else {
      if(positionX < 0.0 || positionY < 0.0) {
        Serial.println("Trying to move while position is not set, please send position before moving.");
      }
      command = IDLE;
    }
  } else if(command == SETUP) {


    if (parameters[PARAMETER_X] > 0) {
      machineWidth = parameters[PARAMETER_X];
    }
    if (parameters[PARAMETER_S] > 0) {
      stepsPerRevolution = parameters[PARAMETER_S];
    }
    if (parameters[PARAMETER_F] > 0) {
      microstepResolution = parameters[PARAMETER_F];
      unsigned int microstepResolutionPower = log(microstepResolution)/log(2);
      setMicrostepResolutionFromPower(microstepResolutionPower, M0_L, M1_L, M2_L, &microstepResolutionL);
      setMicrostepResolutionFromPower(microstepResolutionPower, M0_R, M1_R, M2_R, &microstepResolutionR);
    }
    if (parameters[PARAMETER_P] > 0) {
      millimetersPerRevolution = parameters[PARAMETER_P];
    }

    millimetersPerStep = millimetersPerRevolution / (stepsPerRevolution * microstepResolution);

    SerialPrintln4("SETUP: machineWidth", machineWidth, "stepsPerRevolution", stepsPerRevolution, "millimetersPerRevolution", millimetersPerRevolution, "millimetersPerStep", millimetersPerStep);

    command = IDLE;

  } else if(command == INVERT_XY) {
    invertX = parameters[PARAMETER_X] < 0;
    invertY = parameters[PARAMETER_Y] < 0;

    command = IDLE;

    SerialPrintln2("invertX", invertX, "invertY", invertY);

  } else if(command == SET_PROGRESSIVE_MICROSTEPS) {
    progressiveMicrosteps = parameters[PARAMETER_F] > 0;

    command = IDLE;

    SerialPrintln1("SET_PROGRESSIVE_MICROSTEPS", progressiveMicrosteps);
    
  } else if(command == SET_POSITION) {

    setPosition(parameters[PARAMETER_X], parameters[PARAMETER_Y]);
    command = IDLE;

    SerialPrintln4("SET_POSITION: Position: x", positionX, "y", positionY, "Position: l", positionL, "r", positionR);

  } else if(command == DISABLE_MOTORS) {

    disableMotors();

    SerialPrintln("DISABLE_MOTORS");

    command = IDLE;

  } else if(command == ENABLE_MOTORS) {

    enableMotors();

    SerialPrintln("ENABLE_MOTORS");

    command = IDLE;

  } else if(command == MOVE_PEN) {

    servoAngle = parameters[PARAMETER_S];

    SerialPrintln1("MOVE_PEN", servoAngle);

  } else if(command == RESET) {
    reset();
  }

  if(sendReady) {
    Serial.println("READY");
  }
}

void executeCommand() {

  if (command == MOVE_DIRECT) {
    updateMotorsDirect();
  } else if (command == MOVE_DIRECT_MAX_SPEED) {
    updateMotorsDirectMaxSpeed();
  } else if (command == MOVE_LINEAR) {
    updateMotorsLinear();
  } else if (command == MOVE_PEN) {
    updatePen();
  } else if (command == WAIT) {
    command = IDLE;
    speed = minSpeed;
  } else if (command == IDLE) {
    command = IDLE;
    speed = minSpeed;
  }

}

void reset() {
  Serial.println("RESET");

  command = IDLE;
  nextCommand = IDLE;

  for (unsigned int i = 0 ; i < NUM_PARAMETERS ; i++) {
    parameters[i] = -1.0;
    parametersSet[i] = false;
  }

  parameterIndex = -1;

}




// ----
// Loop
// ----




void loopTwoCommands()
{
  readCommand();

  if (nextCommandReadyToStart() && commandDone()) {
    startCommand(true);
  }

  executeCommand();
}

bool readySent = false;

void loop()
{
  readCommand();

  if (nextCommandReadyToStart() && commandDone()) {
    startCommand(false);
    readySent = false;
  }

  executeCommand();

  if(commandDone() && !readySent) {
    Serial.println("READY");
    readySent = true;
  }
}