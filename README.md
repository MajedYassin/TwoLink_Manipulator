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
#Trajectory Planning 
The Trajectory planning solver implements a trapezoidal trajectory with constrained maximum acceleration and velocity(these are bot parameters that can changed in the bot object using set_acceleration). A Trapezoidal trajectory typically comprises three phases of motion: 

- Constant Acceleration 
- Constant Velocity 
- Constant Deceleration 

However, this assumes that the link motors reach maximum velocity by the end of the acceleratin phase, which will be extremely unlikely for small joint rotations. 

For any rotation equal to less than the square of the maximum velocity over the maximum acceleration, we can expect to see only two phases.  This is demonstrated by figure 1. 

<p align="center">
  <img src="Figures/Link1Trajectoriesexample.jpg">
  
  <b> figure 2: Trajectories of First Link </b><br>
</p>

In this example our first link performs a larger rotation equivalent to approximatel 0.7854 radians.  
As such the motion of the second link will be constrained by the acceleration time of the first link, resulting in a lower constant acceleration in the second link.  This gives the following trajectories for the second link: 

<p align="center">
  <img src="Figures/Link2Trajectoriesexample.jpg">
  
  <b> figure 2: Trajectories of Second Link </b><br>
</p>





