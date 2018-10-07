#include <avr\io.h>

#ifndef BP_PID_H
#define BP_PID_H


class pid{

	       public:

            PID();

            // weighing fctor of the proportional, integral and derivative term
            // Ideally, use a 2nd order DE to approximate these
            void setWeights(const float &Kp, const float &Ki, const float &Kd);

            // Set the maximum and minimum values the algorythim is allowed to output
            void setOutputLowerLimit(const float &output_lower_limit);
            void setOutputUpperLimit(const float &output_upper_limit);

            // MUST USE EVERYTIME BEFORE COMPUTATION
            // udates the data of pilot intention and current position
            void setPoints(const float &desired_point, const float &actual_point);

            // performs the main computation
            // returns the required output
            float refresh();


        private:

        	// computational functions
        	float integrate();
        	float differentiate();
        	float find_proportional();
        	
        	//makes sure integral is not growing infinitly
        	// sets the windup flag if it is
        	void check_for_windup();

            // stores the previous 5 errors, for use in integral, derivativative and current error calculations
            float error[5];
            float Kp, Ki, Kd;

            // I/O
            float set_point;            
            float output;

            // computational results
            float _integral;
			float _derivative;
			float _proportional;

			// limits
            float output_upper_limit;
            float output_lower_limit;

            //windup flag
            bool integral_windup;



};






#endif