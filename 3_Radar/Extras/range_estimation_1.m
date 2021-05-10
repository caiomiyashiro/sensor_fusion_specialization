%{
Range Estimation Exercise
Using the following MATLAB code sample, complete the TODOs to calculate the range in meters of four targets with respective measured beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz].

You can use the following parameter values:

The radar maximum range = 300m
The range resolution = 1m
The speed of light c = 3*10^8
Note : The sweep time can be computed based on the time needed for the signal to travel the maximum range. In general, for an FMCW radar system, the sweep time should be at least 5 to 6 times the round trip time. This example uses a factor of 5.5:

T_{chirp} =5.5\cdot 2 \cdot R_{max}/cT 
chirp
​	 =5.5⋅2⋅R 
max
​	 /c​

Print your answer using the disp function.
%}

c = 3*10^8;
delta_r = 1;
Bsweep = c/(2*delta_r);

range_max = 300;
Ts = 5.5 * (range_max*2/c);

Fb = [0 1.1e6 13e6 24e6]; % beat frequencies in MHz

ranges = c * Ts * Fb / (2*Bsweep)
