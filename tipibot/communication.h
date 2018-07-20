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