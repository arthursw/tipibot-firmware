#pragma once

#include <iostream>

using namespace std;

class Servo {
public:
	int angle;

	Servo() {

	}

	void attach(int pin) {
		cout << "attach servo: " << pin << endl;
		return;
	}

	void write(int angle) {
		this->angle = angle;
		cout << "write to servo: " << angle << endl;
		return;
	}

	int read() {
		return this->angle;
	}
};
