#ifdef SIMULATOR
#include <FakeArduino/Servo.h>
using namespace std;
#else
#include <Arduino.h>
#include <Servo.h>
#endif

// Constants

// Motor driver Pins 
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

const unsigned int SERVO = 4;                               // Servo pin

const unsigned int MIN_MOTOR_PULSE_WIDTH = 1;               // Taken from AccelStepper

const unsigned long TIME_TO_CONSIDER_STOP = 500000L;        // After this amount of microseconds doing nothing: set speed to minSpeed

// Speed is in steps per seconds (step/s)
// Acceleration in steps per seconds^2 (step/s^2) (but this is not really respected)
const float MAX_MOTOR_SPEED = 5 * 200.0 * 32;               // 32 000 = 5 turns per seconds
const float MIN_MOTOR_SPEED = 0.5;

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

int parameterIndex = -1;                                    // The current parameter being parsed

String serialInput = "";                                    // This will be filled with incoming serial characters
                                                            // Using a String in Arduino is not really efficient but it is simple 
                                                            // (good for code readability and maintainance)
                                                            // If the command parsing takes too much time, it is possible to use a fixed char array instead

typedef enum { COMMAND, PARAMETER, VALUE } ReadingType;

ReadingType readingType = COMMAND;

typedef enum {  IDLE, MOVE_DIRECT, MOVE_LINEAR, MOVE_L, MOVE_R, MOVE_PEN, WAIT, SET_POSITION, SET_MAX_SPEED, SET_SERVO_SPEED, SET_FEEDBACK, SET_ACCELERATION, ENABLE_MOTORS, DISABLE_MOTORS, SETUP, INVERT_MOTORS, SET_PROGRESSIVE_MICROSTEPS, RESET } Command;
Command command = IDLE;
Command nextCommand = IDLE;

// timings
unsigned long lastStepTime = 0;               // The last time one motor stepped, used to trigger steps at the right speed
unsigned long delayDuration = 0;              // Used to know how long should we wait during a pause

// Linear steps: the number of steps to do and the number of steps done in a linear step
unsigned long nStepsToDoL = 0;
unsigned long nStepsToDoR = 0;
unsigned long nStepsDoneL = 0;
unsigned long nStepsDoneR = 0;


// Motor directions
bool clockwiseL = true;
bool clockwiseR = true;

// Settings
float machineWidth = 2000;                    // in millimeters
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
float acceleration = 500;                      // See decelerate() to understand why this is not the exact deceleration (not at all)
float servoSpeed = 180;                        // in deg/sec
unsigned long feedbackRate = 100;

// Current state

// Current position and target in (X, Y) coordinates (orthonormal) and (L, R) coordinates (string lengths)
float positionX = -1.0;
float positionY = -1.0;
long positionL = 0;
long positionR = 0;
unsigned int microstepResolutionL = microstepResolution;
unsigned int microstepResolutionR = microstepResolution;


// motors state
bool motorsOn = false;

// Current servo angle
int servoTargetAngle = 0;

Servo servo;

#include "serial.h"
#include "utils.h"
#include "motors.h"
#include "actions.h"
#include "communication.h"
#include "commands.h"

void setup() {

  // Initialize serial communication, pins, the servo and enable motors

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

// This loop is never used
// It execute the commands one after the other, waiting for the current command to be completed before parsing a new one
// If the command takes too much time to be parsed, the motors will stop between two commands
bool readySent = false;
void loopSimple()
{
  // Always check for new commands, important if emergency stop is sent (M0)
  readCommand();

  // If one new command is ready to execute (parsed) and the previous one is done: start the new command
  if (nextCommandReadyToStart() && commandDone()) {
    startCommand(false);
    readySent = false;
  }

  // Always execute current command
  executeCommand();

  // Only send the "Ready" signal (so that the controller sends the next one) when the command is done
  if(commandDone() && !readySent) {
    Serial.println("READY");
    readySent = true;
  }
}

// This loop enables to read a new command while the previous one is not yet finished
// This should avoid the motors to stop while parsing the new command
void loop()
{
  // Always check for new commands, important if emergency stop is sent (M0)
  readCommand();

  // If one new command is ready to execute (parsed) and the previous one is done: start the new command
  if (nextCommandReadyToStart() && commandDone()) {
    startCommand(true);
  }

  // Always execute current command
  executeCommand();
}
