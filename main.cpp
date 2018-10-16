#include <iostream>
#include "bp_PID.h"

using namespace std;

int main(){

	float target;
	float current;

	pid aileron;

	aileron.setWeights(0.5,0.5,0.5);
	aileron.setOutputLimits(-10, 10);

	for (int i = 0; i < 100; i++){

		cin >> current;

		aileron.setPoints(100, current);

		cout<< "motors set to" << aileron.refresh() << endl;

	}

	return 1;
}