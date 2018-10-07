#include "bp_PID.h"


float pid::refresh(const float &feedback_input) {

	// calculate the new output
	output = Kp*find_proportional() + Ki*integrate() + Kd*differentiate();


	// check wether output is maxed in either direction
	if (output < output_lower_limit){
		
		output = output_lower_limit;
	}
	
	else if (output > output_upper_limit){
		
		output = output_upper_limit;
	}

	check_for_windup();
	
			
	return output;
}


float pid::integrate(){

	// since we only compute a new integral every 3 cycles, this var tracks how many cycles since the last integral was delivered
	static int cycles_since = 0;

	float local_integral;

	if (cycles_since != 3){

		cycles_since ++;

		// return the last computed integral
		return _integral;
	}
	
	// avoids increasing the integral indefinitly
	// set by another method if it sees that the quad is not budging
	if (integral_windup == true){

		return _integral;

	}

	// apply simpson's rule
	local_integral = (5 * time_between_calls / 6);
	local_integral *= ( error[0] + (4*error[1]) + error[2] );

	// push the added area to the actual integral variable
	_integral = local_integral;


	cycles_since = 0;

	return _integral;


}

float pid::differentiate(){

	_derivative = 3*error[0] - 4*error[1] + error[2];
	_derivative \= 2*time_between_calls;

	return _derivative; 
}

float pid::find_proportional(){

	_proportional = 0.6*error[0] + 0.4*error[1] + 0.2*error[2] - 0.2*error[4];

	return _proportional; 
}