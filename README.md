# Autopilot-Design-for-a-Quadcopter Implementation
This repository contains source code and simulation files used in my final year project [Autopilot Design for a Quadcopter](https://www.researchgate.net/publication/331298873_Autopilot_Design_for_a_Quadcopter)

## Setup
The simulation files are created using Matlab 2016. To run the simulation open the autopilot_model.slx file and run it. The lineariezed PID contriller is implemented in "Controller block". The quadcopter mathmatical model and dynamics are implemented in 'copter_dynamics.m' file and linked to 'CopterSystem' block. The inputs to the system are the [X,Y,Z] coordinates represented in step input.
The "apm_copter_codes" folder contains the implementation of the designed linear controller on APM_Arducopter written in C++ arduino IDE. 

## Citing

If you use this simulation in your research, you can cite it as follows:

```bibtex
@misc{plappert2016kerasrl,
    author = {Matthias Plappert},
    title = {keras-rl},
    year = {2016},
    publisher = {GitHub},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/keras-rl/keras-rl}},
}
@thesis{thesis,
author = {Hussein, Amir and Abdallah, Rayyan},
year = {2017},
month = {10},
pages = {},
title = {Autopilot Design for a Quadcopter},
doi = {10.13140/RG.2.2.17020.80008}
}

## Contact
In case you faced any issues or found any bugs feel free to contact me at "anh21@mail.aub.edu"
