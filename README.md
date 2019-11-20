## TwoLink_Manipulator Project

# General Info
This project models the motion of a 2DOF two link manipulator. 
The project comprises the following solvers used to estimate the motion and monitor the state of the manipulator: 
- Feedforward torque control;
- Forward Kinematics;
- Inverse Kinematics;
- Velocity Kinematics (calculating end-effector velocity);
- Trajectory planner (trapezoidal trajectory);

# Technology 
software and language used:
- C++ version 11;
- CMake minimum required version 3.14;

Specific tools and external libraries required 

- FetchContent

- googletest GIT_TAG release - 1.8.0
- eigen GIT_TAG - 3.3.7 

The above libraires can be linked to the CMake project using FetchContent as shown in the project CMakeLists.txt. 

# Setup 
To run this project, clone the project directly from the repository address: 
```
$ git clone "https://github.com/MajedYassin/TwoLink_Manipulator.git"
$ cd <cloned repository>
$ mkdir build 
$ cd build
$ cmake ..
$ make -j "number of threads available" 
```
