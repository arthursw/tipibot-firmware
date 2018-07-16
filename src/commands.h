
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

bool commandDone() {
  return command == IDLE;
}

bool nextCommandReadyToStart() {
  return nextCommand != IDLE && readingType == COMMAND;
}

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