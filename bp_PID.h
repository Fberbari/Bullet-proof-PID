#include <avr\io.h>

#ifndef BP_PID_H
#define BP_PID_H


/************************************************
*	This class gathers the pilots intention and the current position of the quad and perfoms the PID algorythim.
*	It then stores integers that represent percentages in the "motor array"
*
*	The integrator levrages simpson's rule to achieve an error of order 3
*	The differentiatiator produces error order 2
*	 the proportional calculation produces error of order 2
************************************************/



class pid{

	       public:

	       	/****************************************************************
			Arguments:
				none

			Error handling:
				none. 

			returns:
				nothing

			Description:


			Notes:
				
            *****************************************************************/
            PID();

            /****************************************************************
			Arguments:
				the weighing factors for the Proportional, Integral, and Derivative terms, respectively

			Error handling:
				none. It is up to the user to provide positive arguments.

			returns:
				nothing

			Description:
				embedds the arguments into private membervariables for use by other functions

			Notes:
				THese will eventually be fine tuned to provide the best response but it is recommended that a second order DE be solved to get close to critcal damping
            *****************************************************************/
            void setWeights(const float &Kp, const float &Ki, const float &Kd);

            /****************************************************************
			Arguments:
				The maximum and minimum values that te controller should output at any given time

			Error handling:
				none. It is up to the user to provide arguments that work with his own reference (wether that is angle, distance etc.)

			returns:
				nothing

			Description:
				c

			Notes:
				
            *****************************************************************/
            void setOutputLowerLimit(const float &output_lower_limit);
            void setOutputUpperLimit(const float &output_upper_limit);

            // MUST USE EVERYTIME BEFORE COMPUTATION
            // udates the data of pilot intention and current position
          	/****************************************************************
			Arguments:
				desired point:	Where the pilot intends the device to be
				actual point:	Where the device currently is

			Error handling:
				none. It is again up to the programmer to ensurecorrect data gets in

			returns:
				nothing

			Description:
				substracts the actual point from the disered point andstores that data into the error array
	
			Notes:
				This must be called every time the loop runs so as to ensure data gets updated real time
            *****************************************************************/
            void setPoints(const float &desired_point, const float &actual_point);

	       	/****************************************************************
			Arguments:
				none

			Error handling:
				none. This function will only run correctly if all the functions above have been used properly
				

			returns:
				the output of the controller

			Description:
				calls the private "computational functions" and combines their results to produce an ouput

			Notes:
				this function checks for INTEGRAL WINDUP
				ensures the output does not cross the bounds set previously
            *****************************************************************/
            float refresh();


        private:

        	// computational functions
        	float integrate();
        	float differentiate();
        	float find_proportional();
        	
        	//makes sure integral is not growing without bound
        	// sets the windup flag if it is
        	void check_for_windup();

            // stores the previous 5 errors, for use in integral, derivativative and current error calculations
            // This array should be accessed circularly
            float error[5];

            // stores the weight constants
            float Kp, Ki, Kd;

            // I/O
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