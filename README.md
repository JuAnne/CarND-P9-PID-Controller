# CarND-P9-PID-Controller

Udacity Self-Driving Car Nanodegree - PID Controller Project Implementation

## Overview

PID is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name.

This project consists of implementing a PID controller with C++.  As mentioned in the lecture, D is useful for steering drift correction and I is useful for systematic bias correction. A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates  cross-track error (CTE), speed, and steering angle data. And the PID controller responds with steering and throttle commands driving the car around the similated track. The communication between the simulator and the PID controller is done using the [uWebSockets](https://github.com/uNetworking/uWebSockets). To get this project started, Udacity provides a seed project that could be found [here](https://github.com/udacity/CarND-PID-Control-Project).


## Prerequisites
The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

This particular implementation is done on Linux OS. In order to install the necessary libraries, use the install-ubuntu.sh.

## Compiling and executing the project
These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `make`: This will create an executable `particle_filter`


## Parameters tuning 
I started by trying to tune the parameters manually:
1. To get a feel for how the system reacts to each parameter and their respective sensitivities. 
2. A good starting point for improvement, such as "twiddle".

My evaluation of a given set of parameters was based on:
1. Visual inspection of how the car went around the track, did it oscillate, how did it handle the tight corners?
2. Plots of cte and steering value, plots of P controller, PD controller and PID controller for comparison.

I also implemented a simple throttle controller to slow down the vehicle if the steering angle is considered high or if the cross track error is high.

After I manually tuned the parameters to a fairly good condition, I switched to try "twiddle" algorithm for fine tuning. The parameters generated didn't seems to make a big difference on the performance and it also coverged very slowly. The parameters of my final implementation are those manually tuned.

## Visualization
Here are my sample plots of cte-steer and P-PD-PID controller
![](https://github.com/JuAnne/CarND-P9-PID-Controller/blob/master/plot/cte_steer.png)
![](https://github.com/JuAnne/CarND-P9-PID-Controller/blob/master/plot/p_pd_pid_2.png)

My output video can be downloaded ([here](https://github.com/JuAnne/CarND-P9-PID-Controller/blob/master/pid.mp4))





