# TwoLink_Manipulator

#General Info
This project model simulates the motion of a simple 2DOF two link manipulator. 
The project comprises the following solvers used to estimate the motion and behaviour of the manipulator: 
- Feedforward torque control;
- Forward and Inverse Kinematics;
- Velocity Kinematics (calculatign end-effector velocity);
- Trajectory planer (trapezoidal trajectory);

#Technology 

software and language used:
- C++ version 11;
- Cmake minimum required version 3.0;

Specific tools and external libraries required 

- FetchContent
- googletest 
- eigen

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
