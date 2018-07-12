/*
  DSM2_tx implements the serial communication protocol used for operating
  the RF modules that can be found in many DSM2-compatible transmitters.

  Copyrigt (C) 2012  Erik Elmore <erik@ironsavior.net>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

class FakeSerial {

public:
  string fakeInput;
  string fakeOutput;
  int dataIndex;
  bool ignoreSerial;

	FakeSerial():
    fakeInput(""),
    fakeOutput(""),
    dataIndex(0)
  {

	}

	void fakeFeed(string& testCommands) {
		fakeInput += testCommands;
	}

	bool available() {
		return fakeInput.length() > 0;
	}

	char read() {
		#ifdef PRINT_SERIAL_READ
		if(fakeInput == "\n") {
			cout << " <- read";
		}
		cout << " serial in: \t \t" << fakeInput;
		#endif
    char firstChar = fakeInput.at(0);
    fakeInput.erase(0, 1);
		return firstChar;
	}

  size_t write(const string& message){
    fakeOutput += message;
    // cout << "\t\twrite to serial: " << fakeOutput << endl;
    #ifdef OVERRIDE_WRITE_SERIAL
    if(!ignoreSerial) {
      fakeWriteSerial(fakeOutput);
    }
    #endif
    size_t fakeOutputLength = fakeOutput.length();
    fakeOutput = "";
    return fakeOutputLength;
  }

  size_t write(const char* data, size_t length) {
    string message(data, length);
    write(message);
    return length;
  }

  void writeNoSend(const string& message) {
    fakeOutput += message;
  }

  void println(const char * str)
  {
    string message(str);
    write(message);
  }

  void print(const char * str)
  {
    writeNoSend(str);
  }
  void println(float str)
  {
    stringstream ss (stringstream::in | stringstream::out);
    ss << str;
    write(ss.str());
  }
  void print(float str)
  {
    stringstream ss (stringstream::in | stringstream::out);
    ss << str;
    writeNoSend(ss.str());
  }
  void println(std::string str)
  {
    write(str);
  }
  void print(std::string str)
  {
    writeNoSend(str);
  }
  int readBytesUntil(const char c, const char * buffer, int bufferSize){
    return 1;
  }
  void begin(unsigned long speed){

  }

  char peek() {
	  return 1;
  }

  void end(){

  }
};

extern FakeSerial Serial;
