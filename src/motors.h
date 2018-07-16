// Microsteps
unsigned int nMicroStepsL = 0;
unsigned int nMicroStepsR = 0;

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
// (that is why invertMotorL and invertMotorR work in opposite ways)
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
  bool clockwise = invertMotorL != clockwiseL;
  step(DIRECTION_L, STEP_L, clockwise, &nMicroStepsL, microstepResolutionL);
  positionL += clockwiseL ? 1 : -1;
  nStepsDoneL++;
}

void stepR()
{
  bool clockwise = invertMotorR != clockwiseR;
  step(DIRECTION_R, STEP_R, !clockwise, &nMicroStepsR, microstepResolutionR);
  positionR += clockwiseR ? 1 : -1;
  nStepsDoneR++;
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
