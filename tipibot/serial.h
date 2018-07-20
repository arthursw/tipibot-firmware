#define ENABLE_SERIAL

void SerialPrint(const char* message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(const char* message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrint(float message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(float message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrint(int message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(int message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrint(unsigned int message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(unsigned int message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrint(long message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(long message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrint(unsigned long message) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  #endif
}

void SerialPrintln(unsigned long message) {
  #ifdef ENABLE_SERIAL
  Serial.println(message);
  #endif
}

void SerialPrintln1(const char* message, float value) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

void SerialPrintln1(const char* message, int value) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

void SerialPrintln1(const char* message, unsigned int value) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

void SerialPrintln1(const char* message, long value) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

void SerialPrintln1(const char* message, unsigned long value) {
  #ifdef ENABLE_SERIAL
  Serial.print(message);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

void SerialPrintln2(const char* message1, float value1, const char* message2, float value2) {
  #ifdef ENABLE_SERIAL
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  #endif
}

void SerialPrintln2(const char* message1, int value1, const char* message2, int value2) {
  #ifdef ENABLE_SERIAL
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  #endif
}

void SerialPrintln2(const char* message1, unsigned int value1, const char* message2, unsigned int value2) {
  #ifdef ENABLE_SERIAL
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  #endif
}

void SerialPrintln2(const char* message1, long value1, const char* message2, long value2) {
  #ifdef ENABLE_SERIAL
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  #endif
}

void SerialPrintln2(const char* message1, unsigned long value1, const char* message2, unsigned long value2) {
  #ifdef ENABLE_SERIAL
  Serial.print(message1);
  Serial.print(": ");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(message2);
  Serial.print(": ");
  Serial.println(value2);
  #endif
}

void SerialPrintln4(const char* message1, float value1, const char* message2, float value2, const char* message3, float value3, const char* message4, float value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}

void SerialPrintln4(const char* message1, float value1, const char* message2, float value2, const char* message3, long value3, const char* message4, long value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}

void SerialPrintln4(const char* message1, int value1, const char* message2, int value2, const char* message3, int value3, const char* message4, int value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}

void SerialPrintln4(const char* message1, unsigned int value1, const char* message2, unsigned int value2, const char* message3, unsigned int value3, const char* message4, unsigned int value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}

void SerialPrintln4(const char* message1, long value1, const char* message2, long value2, const char* message3, long value3, const char* message4, long value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}

void SerialPrintln4(const char* message1, unsigned long value1, const char* message2, unsigned long value2, const char* message3, unsigned long value3, const char* message4, unsigned long value4) {
  #ifdef ENABLE_SERIAL
  SerialPrintln2(message1, value1, message2, value2);
  SerialPrintln2(message3, value3, message4, value4);
  #endif
}
