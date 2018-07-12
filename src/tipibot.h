#ifdef SIMULATOR
#include <FakeArduino/Servo.h>
using namespace std;
#else
#include <Arduino.h>
#include <Servo.h>
#endif

#define DEBUG

#include "debug.h"
#include "utils.h"
#include "motors.h"

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

const unsigned long TIME_TO_CONSIDER_STOP = 500000L;        // After this amount of microseconds doing nothing: set speed to minSpeed

// Speed is in steps per seconds (step/s)
// Acceleration in steps per seconds^2 (step/s^2)
const float MAX_MOTOR_SPEED = 5 * 200.0 * 32;               // 32 000 = 5 turns per seconds
const float MIN_MOTOR_SPEED = 10;

// See decelerate()
const float ACCELERATION_FACTOR = 10.0;


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

typedef enum {  IDLE, MOVE_DIRECT, MOVE_LINEAR, MOVE_L, MOVE_R, MOVE_PEN, WAIT, SET_POSITION, SET_MAX_SPEED, SET_SERVO_SPEED, SET_FEEDBACK, SET_ACCELERATION, ENABLE_MOTORS, DISABLE_MOTORS, SETUP, INVERT_MOTORS, SET_PROGRESSIVE_MICROSTEPS, RESET } Command;
Command command = IDLE;
Command nextCommand = IDLE;

// timings
unsigned long lastStepTime = 0;
unsigned long delayDuration = 0;

// Linear steps
unsigned long nStepsToDoL = 0;
unsigned long nStepsToDoR = 0;
unsigned long nStepsDoneL = 0;
unsigned long nStepsDoneR = 0;
unsigned long nLinearStepsToDo = 0;
unsigned long nLinearStepsDone = 0;

// Microsteps
unsigned int nMicroStepsL = 0;
unsigned int nMicroStepsR = 0;

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
bool invertMotorL = false;
bool invertMotorR = false;

// Speed and acceleration in steps per seconds
float maxSpeed = 200 * 32;
float minSpeed = MIN_MOTOR_SPEED;
float speed = minSpeed;
float acceleration = 500;
float servoSpeed = 180;                        // in deg/sec
unsigned long feedbackRate = 100;

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

bool deceleration = false;

// motors state
bool motorsOn = false;

// Current servo angle
int servoTargetAngle = 0;

Servo servo;

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

  Serial.println("Tipibot ready!");
  Serial.println("Initialize");
}


// -------
// Actions
// -------


void accelerate(unsigned long deltaTime) {
  // Use the proper acceleration forumla, deceleration is handled differently
  // See decelerate() form more information
  speed += acceleration * (deltaTime / 1000000.0);
  // speed += acceleration / speed;
  speed = min(speed, maxSpeed);
}

void decelerate(unsigned long deltaTime) {
  // Note: The speed is quantified (updated every deltaTime microseconds, deltaTime being inversely proportional to the speed)
  //       This prevent from using the simple formula: speed -= acceleration * (deltaTime / 1000000.0)
  //       Since an error will accumulate, the actual time to stop will be much larger than the theoretical time to stop
  //       In a normal orthogonal space, a nice technique to compute speed is described here: http://u.to/ScipEg
  //       Since we are not in an orthogonal space, we cannot apply this method: 
  //       the speed of motors is not linearly proportionnal to the speed of the pen
  //       To use the proper formula, we need to know the exact braking distance / required amount of time to stop ;
  //       this is not simple due to both the quantification and the conversion from orthogonal to polar space.
  //       Instead, we just use a nice slope (ACCELERATION_FACTOR / x) to gently break until we stop
  //       ACCELERATION_FACTOR is there to ensure deceleration is fast enough to fully stop the motors in the given time
  //       this is not really elegant, but having the perfect theoretical acceleration / deceleration is quite complex
  //       The computed distance to stop might be larger than the actual distance to stop, that is why we keep the 'decelerate' boolean
  //       to tell if we are decelerating, in which case we must not accelerate anymore (or it could oscillate between accelerations and decelerations)

  // speed -= acceleration * (deltaTime / 1000000.0);
  speed -= ACCELERATION_FACTOR * acceleration / speed;
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

  nStepsDoneL = 0;
  nStepsDoneR = 0;

  nStepsToDoL = abs(deltaL);
  nStepsToDoR = abs(deltaR);

  if (nStepsToDoL == 0 && nStepsToDoR == 0) {
    // Reached position
    Serial.println("Error: trying to set speed while reached position.");
    return;
  }

  // Note: this might not be the best place to compute progressive microsteps since speed is not really related to nStepsToDo
  if(progressiveMicrosteps) {
    computeProgressiveMicrostep(nStepsToDoL, nStepsToDoR);
  }

  // SerialPrintln4("--- computeStepsToDo: nStepsToDo: l", nStepsToDoL, "r", nStepsToDoR, "clockwise: l", (unsigned long)clockwiseL, "r", (unsigned long)clockwiseR);
  // SerialPrintln4("--- computeStepsToDo: subTarget: l", subTargetL, "r", subTargetR, "position: l", positionL, "r", positionR);
}

// Compute global speed in orthogonal space (x, y - mm) from acceleration
void computeSpeed(unsigned long deltaTime) {

  // Note: see decelerate() for explanations about acceleration and decelaration

  // unsigned long nStepsToStop = speed * speed / (2.0 * acceleration);       // Simple version when minSpeed == 0
  
  unsigned long nStepsToStop = ( (speed - minSpeed) / acceleration ) * ( (speed + minSpeed) / 2.0 );

  // For linear moves: problem with orthogonal distance: we often need less steps than expected since both motors run
  // This results in over estimating the distance left
  // it will not do as many steps so it will not have time to break

  // float deltaX = targetX - positionX;
  // float deltaY = targetY - positionY;

  // float distance = sqrt(deltaX * deltaX + del taY * deltaY);
  // nStepsLeft = floor(distance / millimetersPerStep);

  // Use max nStep distance: 
  //  this is dangerous: the total number of steps for one motor can appear to be 0, but it could be much more when going on a line
  //  however, we admit that the motor which will have the more steps to do will never have more actual steps than appearing steps
  //  (this is yet to be proved)

  unsigned long deltaL = abs(positionL - targetL);
  unsigned long deltaR = abs(positionR - targetR);

  unsigned long nStepsLeft = max(deltaL, deltaR);
  
  if(nStepsLeft < nStepsToStop && speed > minSpeed) {

    deceleration = true;
    decelerate(deltaTime);

    // cout << "decelerate ";

  } else if(speed < maxSpeed && !deceleration) {
    
    accelerate(deltaTime);

    // cout << "accelerate ";
  }
  
  // cout << "nStepsLeft: " << nStepsLeft << ", nStepsToStop: " << nStepsToStop << ", speed: " << speed << ", acceleration: " << acceleration << endl;

  // SerialPrintln2("nStepsLeft", nStepsLeft, "nStepsToStop", nStepsToStop, true);
  // SerialPrintln2("speed", speed, "acceleration", acceleration, true);

  // cout << "speed: " << speed << endl;
}

// updates

void updatePen() {

  unsigned long time = micros();
  unsigned long deltaTime = time - lastStepTime;
  
  float period = 1000000L / servoSpeed;

  if(deltaTime < period) {
    return;
  }

  lastStepTime = time;
  
  int currentAngle = servo.read();
  bool clockwise = servoTargetAngle - currentAngle > 0;
  
  servo.write(currentAngle + (clockwise ? 1 : -1) );

  if(servo.read() == servoTargetAngle) {
    command = IDLE;
  }

  // cout << "servoSpeed: " << servoSpeed << ", currentAngle: " << currentAngle << ", servoTargetAngle: " << servoTargetAngle << ", clockwise: " << clockwise << endl;
}

void updateMotors(bool mustComputeSpeed = true) {

  unsigned long time = micros();
  unsigned long deltaTime = time - lastStepTime;
  
  float period = 1000000L / speed;


  if(deltaTime < period) {
    return;
  }

  lastStepTime = time;

  if(mustComputeSpeed) {
    computeSpeed(deltaTime);
  }

  unsigned long nStepsToDoFastest = max(nStepsToDoL, nStepsToDoR);
  unsigned long nStepsDoneFastest = max(nStepsDoneL, nStepsDoneR);

  unsigned long nStepsToDoSlowest = min(nStepsToDoL, nStepsToDoR);
  unsigned long nStepsDoneSlowest = min(nStepsDoneL, nStepsDoneR);
  
  if(nStepsDoneFastest < nStepsToDoFastest) {
    stepFastestMotor();
  }

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

  // unsigned long halfNStepsToDoSlowestMotor = min(nStepsToDoL, nStepsToDoR) / 2;
  // if(nStepsDone < halfNStepsToDoSlowestMotor || nStepsDone >= nStepsToDo - halfNStepsToDoSlowestMotor) {
  //   stepSlowestMotor();
  // }

  if(nStepsDoneFastest * nStepsToDoSlowest / nStepsToDoFastest > nStepsDoneSlowest && nStepsDoneSlowest < nStepsToDoSlowest) {
    stepSlowestMotor();
  }

  polarToOrtho(positionL, positionR, &positionX, &positionY);
}

void checkTargetReached() {

  bool reachedTarget = positionL == subTargetL && positionR == subTargetR;
  
  printPositions(reachedTarget);

  if(reachedTarget) {
    command = IDLE;
  }
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

  // SerialPrintln4("-st: l", subTargetL, "r", subTargetR, "position l", positionL, "r", positionR);
}

void updateMotorsLinear() {
  updateMotors();

  bool reachedTarget = positionL == subTargetL && positionR == subTargetR;
  printPositions(reachedTarget);

  if (reachedTarget) {
    nLinearStepsDone++;
    if (nLinearStepsDone >= nLinearStepsToDo) {                   // nLinearStepsToDo can be 0, in which case nLinearStepsDone > nLinearStepsToDo
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
  lastStepTime = micros();

  deceleration = false;
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
        nextCommand = INVERT_MOTORS;
      } else if(serialInput == "M13") {
        nextCommand = SET_PROGRESSIVE_MICROSTEPS;
      } else if(serialInput == "M14") {
        nextCommand = SET_SERVO_SPEED;
      } else if(serialInput == "M15") {
        nextCommand = SET_FEEDBACK;
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

void startCommand(bool sendReady) {
  
  command = nextCommand;
  nextCommand = IDLE;

  // TODO: handle errors in parameters

  if(command == MOVE_DIRECT || command == MOVE_LINEAR) {

    if(parametersSet[PARAMETER_F]) {
      
      maxSpeed = min(parameters[PARAMETER_F], MAX_MOTOR_SPEED);

      SerialPrintln1("SET_MAX_SPEED: maxSpeed", maxSpeed);
    }

    if(parametersSet[PARAMETER_S]) {

      acceleration = parameters[PARAMETER_S];

      SerialPrintln1("SET_ACCELERATION: acceleration", acceleration);
    }

    if(parametersSet[PARAMETER_P]) {
      
      minSpeed = max(parameters[PARAMETER_P], MIN_MOTOR_SPEED);

      SerialPrintln1("SET_MIN_SPEED: minSpeed", minSpeed);
    } else {
      minSpeed = MIN_MOTOR_SPEED;
    }

    if((parametersSet[PARAMETER_X] || parametersSet[PARAMETER_Y]) && positionX >= 0.0 && positionY >= 0.0) {

      float targetX = parametersSet[PARAMETER_X] ? parameters[PARAMETER_X] : positionX;
      float targetY = parametersSet[PARAMETER_Y] ? parameters[PARAMETER_Y] : positionY;

      if(command == MOVE_DIRECT) {
        setTarget(targetX, targetY);
      } else if(command == MOVE_LINEAR) {
        setTargetLinear(targetX, targetY);
      }
      
    } else {
      if(positionX < 0.0 || positionY < 0.0) {
        Serial.println("Trying to move while position is not set, please send position before moving.");
      }
      command = IDLE;
    }
  } else if(command == WAIT) {
    unsigned long delayMilliseconds = parameters[PARAMETER_P];
    SerialPrintln1("Wait: ", delayMilliseconds);
    // cout << "Time to restart: " << (micros() + delayMilliseconds * 1000L) << endl;
    delayDuration = micros() + delayMilliseconds * 1000L;
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

  } else if(command == INVERT_MOTORS) {
    invertMotorL = parameters[PARAMETER_X] < 0;
    invertMotorR = parameters[PARAMETER_Y] < 0;

    command = IDLE;

    SerialPrintln2("invertMotorL", invertMotorL, "invertMotorR", invertMotorR);

  } else if(command == SET_PROGRESSIVE_MICROSTEPS) {
    progressiveMicrosteps = parameters[PARAMETER_F] > 0;

    command = IDLE;

    SerialPrintln1("SET_PROGRESSIVE_MICROSTEPS", progressiveMicrosteps);
    
  } else if(command == SET_SERVO_SPEED) {
    
    servoSpeed = parameters[PARAMETER_F];

    command = IDLE;

    SerialPrintln1("SET_SERVO_SPEED", servoSpeed);

  } else if(command == SET_FEEDBACK) {
    
    feedbackRate = (unsigned long)(parameters[PARAMETER_F]);

    command = IDLE;

    SerialPrintln1("SET_FEEDBACK", feedbackRate);

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

    servoTargetAngle = parameters[PARAMETER_S];
    lastStepTime = micros();

    SerialPrintln1("MOVE_PEN", servoTargetAngle);

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
  } else if (command == MOVE_LINEAR) {
    updateMotorsLinear();
  } else if (command == MOVE_PEN) {
    updatePen();
  } else if (command == WAIT) {
    // cout << "wait: " << micros() <<  ", delay duration: " << delayDuration << endl;
    if(micros() > delayDuration) {
      command = IDLE;
      delayDuration = 0;
    }
    minSpeed = MIN_MOTOR_SPEED;
    speed = minSpeed;
  } else if (command == IDLE) {

    // if tipibot is IDLE more than half of a second: consider it has stopped: speed = minSpeed
    unsigned long time = micros();
    unsigned long deltaTime = time - lastStepTime;

    if(deltaTime > TIME_TO_CONSIDER_STOP) {
      minSpeed = MIN_MOTOR_SPEED;
      speed = minSpeed;
    }

    command = IDLE;
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

// unsigned long timeWhenBeganIDLE = 0;

void loop()
{
  bool idle = command == IDLE;

  readCommand();

  if (nextCommandReadyToStart() && commandDone()) {
    startCommand(true);
  }

  executeCommand();

  // if(!idle && command == IDLE) {    // start IDLE
  //   timeWhenBeganIDLE = micros();
  // }

  // if(idle && command != IDLE && timeWhenBeganIDLE > 0) {
  //   cout << "IDLE time in millis: " << ( (micros() - timeWhenBeganIDLE) ) << endl;
  //   timeWhenBeganIDLE = 0;
  // }
}

bool readySent = false;

void loopSimple()
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