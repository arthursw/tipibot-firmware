unsigned long nLinearStepsToDo = 0;
unsigned long nLinearStepsDone = 0;

bool deceleration = false;

float startX = 0.0;
float startY = 0.0;
float targetX = 0.0;
float targetY = 0.0;
long targetL = 0L;
long targetR = 0L;
long subTargetL = 0L;
long subTargetR = 0L;

unsigned long lastFeedbackTime = 0;

void printPositions(bool force = false) {
  if(feedbackRate == 0) {
    return;
  }

  unsigned long time = micros();
  unsigned long deltaTime = time - lastFeedbackTime;
  
  float period = 1000000L / feedbackRate;

  if(!force && deltaTime < period) {
    return;
  }

  lastFeedbackTime = time;

  SerialPrintln2("-p: l", positionL, "r", positionR);
}

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
    SerialPrintln1("-pen", servoTargetAngle);
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

  if(nStepsToDoFastest > 0 && nStepsDoneFastest * nStepsToDoSlowest / nStepsToDoFastest > nStepsDoneSlowest && nStepsDoneSlowest < nStepsToDoSlowest) {
    stepSlowestMotor();
  }

  polarToOrtho(positionL, positionR, &positionX, &positionY);
}

void updateMotorsDirect() {
  updateMotors();

  bool reachedTarget = positionL == subTargetL && positionR == subTargetR;
  
  printPositions(reachedTarget);

  if(reachedTarget) {
    command = IDLE;
  }
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
  printPositions();

  bool reachedTarget = positionL == subTargetL && positionR == subTargetR;

  if (reachedTarget) {
    nLinearStepsDone++;
    if (nLinearStepsDone >= nLinearStepsToDo) {                   // nLinearStepsToDo can be 0, in which case nLinearStepsDone > nLinearStepsToDo
      printPositions(true);
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
  printPositions(true);
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
