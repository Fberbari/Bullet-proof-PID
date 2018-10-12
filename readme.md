There is an abundance of PID classes on Github. But very few of them leverage algorythims that are robust enough to make it through sensor uncertainty and abherrant points.  

This project is to build an incredibly robust PID class that uses algorythims that produce known error terms that are reasonable enough to produce trustworthy results in systems with a reasonable ammount of momentum

It's main objective is to act as an upgrade on my AAquad flight computer, but it may be used anywhere one sees fit.


This class will work best for systems with larger momentum, but the uncertainties associated with computation of the derivative and integral are as follows:

-The uncertainty on (both the derivative and the proportional terms) is proportional to the upper bound of the 3rd derivative on the interval of interest AND to the square of the time between measurements.

-The uncertainty on the integral is negligable compared to the derivative, as it is proportional to a thousandth of the upper bound of the 4th derivative on the 	interval in question and to the 5th power of the width (time difference) of the integrated interval.



To ensure that damping is as close to critical as possibble, the second order ODE should be modeled to return the correct value of the coefficients ki, kd, kp





THIS CLASS WILL ONLY FUNCTION IF IT'S COMPUTE FUNCTION IS CALLED AT A REGULAR INTERVAL.
USE A TIMER INTERRUPT TO DO SO AND DEFINE THE "time_between_calls" MACRO IN THE "bp_PID.h" FILE 




THe algorithims used to compute the error, it's integral and it's derivative are based on methods seen in ECE 204 at the university of Waterloo, Taught by Prof. Douglas Harder