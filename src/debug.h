#pragma once

// --------
// Debug
// --------


void setIgnoreSerial(bool ignoreSerial = false) {
  #ifdef SIMULATOR
  Serial.ignoreSerial = ignoreSerial;
  #endif
}

void SerialPrint(const char* message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(const char* message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrint(float message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(float message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrint(int message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(int message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrint(unsigned int message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(unsigned int message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrint(long message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(long message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrint(unsigned long message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln(unsigned long message, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.println(message);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln1(const char* message, float value, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln1(const char* message, int value, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln1(const char* message, unsigned int value, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln1(const char* message, long value, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln1(const char* message, unsigned long value, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln2(const char* message1, float value1, const char* message2, float value2, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln2(const char* message1, int value1, const char* message2, int value2, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln2(const char* message1, unsigned int value1, const char* message2, unsigned int value2, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln2(const char* message1, long value1, const char* message2, long value2, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln2(const char* message1, unsigned long value1, const char* message2, unsigned long value2, bool ignoreSerial = false) {
  #ifdef DEBUG
  setIgnoreSerial(ignoreSerial);
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  setIgnoreSerial(false);
  #endif
}

void SerialPrintln4(const char* message1, float value1, const char* message2, float value2, const char* message3, float value3, const char* message4, float value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

void SerialPrintln4(const char* message1, float value1, const char* message2, float value2, const char* message3, long value3, const char* message4, long value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

void SerialPrintln4(const char* message1, int value1, const char* message2, int value2, const char* message3, int value3, const char* message4, int value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

void SerialPrintln4(const char* message1, unsigned int value1, const char* message2, unsigned int value2, const char* message3, unsigned int value3, const char* message4, unsigned int value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

void SerialPrintln4(const char* message1, long value1, const char* message2, long value2, const char* message3, long value3, const char* message4, long value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

void SerialPrintln4(const char* message1, unsigned long value1, const char* message2, unsigned long value2, const char* message3, unsigned long value3, const char* message4, unsigned long value4, bool ignoreSerial = false) {
  #ifdef DEBUG
  SerialPrintln2(message1, value1, message2, value2, ignoreSerial);
  SerialPrintln2(message3, value3, message4, value4, ignoreSerial);
  #endif
}

extern float positionX;
extern float positionY;
extern long positionL;
extern long positionR;
extern unsigned long feedbackRate;


unsigned long lastFeedbackTime = 0;

void printPositions(bool force = false, bool ignoreSerial = false) {
  #ifdef DEBUG
  
  if(feedbackRate == 0) {
    return;
  }

  unsigned long time = micros();
  unsigned long deltaTime = time - lastFeedbackTime;
  
  float period = 1000000L / feedbackRate;

  if(deltaTime < period) {
    return;
  }

  lastFeedbackTime = time;

  SerialPrintln2("-p: l", positionL, "r", positionR, ignoreSerial);

  #endif
}
