# Autopilot-Design-for-a-Quadcopter Implementation
This repository contains source code and simulation files used in my final year project [Autopilot Design for a Quadcopter](https://www.researchgate.net/publication/331298873_Autopilot_Design_for_a_Quadcopter)

## Setup
The simulation files are created using Matlab 2016. To run the simulation open the autopilot_model.slx file and run it. The lineariezed PID contriller is implemented in "Controller block". The quadcopter mathmatical model and dynamics are implemented in 'CopterSystem' block and linked to 'copter_dynamics.m'. The inputs to the system are the [X,Y,Z] coordinates represented in step input.

