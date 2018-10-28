// Websocket communication taken from the easywsclient example

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

// Override serial writes to send messages to websocket instead
#define OVERRIDE_WRITE_SERIAL

void fakeWriteSerial(std::string message)
{
    webs->send(message);
}

#include "FakeArduino/FakeArduino.h"

#define SIMULATOR

#include "../tipibot/tipibot.h"

void handle_message(const std::string & message)
{
	std::string m = message;
	Serial.fakeFeed(m);
}

// Initialize websocket client and continuously call tipibot loop() until websocket is closed
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
