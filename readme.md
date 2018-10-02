 NOT COMPLETED 



This project is to build an incredibly robust PID class.

It's main objective is to act as an upgrade on my AAquad flight computer, but it may be used anywhere one sees fit.


This class will work best for systems with larger momentum, but the uncertainties associated with computation of the derivative and integral are as follows:

-The uncertainty on the derivative is proportional to the upper bound of the 3rd derivative on the interval of interest AND to the square of the timebetween 		measurements.
-The uncertainty on the integral is negligable compared to the derivative, as it is proportional to a thousandth of the upper bound of the 4th derivative on the 	interval in question and to the 5th power of the width (time difference) of the integrated interval.



THIS CLASS WILL ONLY FUNCTION IF IT'S COMPUTE FUNCTION IS CALLED AT A REGULAR INTERVAL.
USE A TIMER INTERRUPT TO DO SO




THe algorithims used to compute the error, it's integral and it's derivative are based on methods seen in ECE 204 at the university of Waterloo, Taught by Prof. Douglas Harder