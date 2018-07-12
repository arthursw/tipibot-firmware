#include "FakeArduino.h"

#include <iostream>
using namespace std;

void randomSeed(unsigned long seed)
{
  if (seed != 0) {
    srandom(seed);
  }
}

long random(long howbig)
{
  if (howbig == 0) {
    return 0;
  }
  return random() % howbig;
}

long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }
  long diff = howbig - howsmall;
  return random(diff) + howsmall;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

auto start = std::chrono::high_resolution_clock::now();

void delay(unsigned long ms) {

}
void delayMicroseconds(unsigned int us) {

}
unsigned long millis() {
	return micros()/1000;
}
unsigned long micros() {
	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	return std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
}

void initialize_mock_arduino() {

}

void pinMode(unsigned char pin, unsigned char mode) {

}

#ifndef OVERRIDE_DIGITAL_WRITE
void digitalWrite(unsigned char pin, unsigned char value) {

}


#endif

// use byte == ' ' instead of isSpace (I don't know exactly which space isSpace considers, but not only ' ' ; '\n' is also considered as space...)
// bool isSpace(int byte) {
//   return byte == ' ';
// }
