#include "easywsclient.hpp"

#ifdef _WIN32
#pragma comment( lib, "ws2_32" )
#include <WinSock2.h>
#endif
#include <assert.h>
#include <stdio.h>
#include <string>
#include <sstream>

using easywsclient::WebSocket;
static WebSocket::pointer webs = NULL;


#define OVERRIDE_WRITE_SERIAL

void fakeWriteSerial(std::string message)
{
    webs->send(message);
}


#include "FakeArduino/SD.h"


void digitalWrite(int pin, int value);

#define SIMULATOR
#define OVERRIDE_DIGITAL_WRITE
#define PRINT_SERIAL_READ
#define DEBUG

#include "../tipibot/tipibot.h"

int motorDirectionL = 0;
int motorDirectionR = 0;

float testPositionL = 0;
float testPositionR = 0;
float testPositionX = 0;
float testPositionY = 0;


void digitalWrite(int pin, int value) {
    if(pin == DIRECTION_L) {
        motorDirectionL = value == 0 ? -1 : 1;
    }
    if(pin == DIRECTION_R) {
        motorDirectionR = value == 0 ? -1 : 1;
    }
    if(pin == STEP_L && value == 0) {
        testPositionL += millimetersPerStep * motorDirectionL;
    }
    if(pin == STEP_R && value == 0) {
        testPositionR += millimetersPerStep * motorDirectionR;
    }
    if(pin == STEP_L || pin == STEP_R){
        polarToOrtho(long(testPositionL), long(testPositionR), &testPositionX, &testPositionY);
    }
}


std::string testCommands = "G4 P0\nM340 P3 S0\nM340 P3 S1500\nG92 X1150.0512 Y1975.0\nG0 X1125.0 Y1900.0\nG90\n";




void handle_message(const std::string & message)
{
	std::string m = message;
	Serial.fakeFeed(m);
}

int main()
{
#ifdef _WIN32
    INT rc;
    WSADATA wsaData;

    rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (rc) {
        printf("WSAStartup Failed.\n");
        return 1;
    }
#endif

    webs = WebSocket::from_url("ws://localhost:6843");
    assert(webs);
	
    setup();
    // Serial.fakeFeed(testCommands);

    while (webs->getReadyState() != WebSocket::CLOSED) {
      webs->poll();
      webs->dispatch(handle_message);
      loop();
    }
    delete webs;
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
