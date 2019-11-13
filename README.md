# TwoLink_Manipulator

#General Info
This project model simulates the motion of a 2DOF two link manipulator. 
The project comprises the following solvers used to estimate the motion and behaviour of the manipulator: 
- Feedforward torque control;
- Forward and Inverse Kinematics;
- Velocity Kinematics (calculatign end-effector velocity);
- Trajectory planer (trapezoidal trajectory);

#Technology 
software and language used:
- C++ version 11;
- Cmake minimum required version 3.14;

Specific tools and external libraries required 

- FetchContent

- googletest GIT_TAG release - 1.8.0
- eigen GIT_TAG - 3.3.7 

The above libraires can be linked to the Cmake project using FetchContent   

#Setup 
To run this project, you can clone th project directly from the repository address 
```
$ git clone "https://github.com/MajedYassin/TwoLink_Manipulator.git"
$ cd "cloned repository"
$ mkdir build //create a build in which to save the Cmake project build files 
$ cd build
$ cmake ..
$ make -j "number of threads available" 
```
