# Model Predictive Control Project

https://youtu.be/NGlVQ3fEfiE

In this project, I have implemented a Model Predictive Controller for controlling the steering and throttle of the car in simulator, such that it closely follows a simulator generated trajectory.


## Setup
Dependencies:
 1. cmake >= 3.5
 2. make >= 4.1
 3. gcc/g++ >= 5.4
 4. uWebSockets: run `install-ubuntu.sh`.
 5. Ipopt and CppAD: Refer <a href="https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md">this document</a> for instructions.
 
 This project involves the Term 2 Simulator which can be downloaded <a href="https://github.com/udacity/self-driving-car-sim/releases">here</a>.
 
 To build the project run the following commands after cloning this repository:
  1. cd build
  2. sudo chmod +x run.sh
  3. ./run

For more info, check <a href="https://github.com/udacity/CarND-MPC-Project">this repository</a>.

## The Model
In MPC we predict the next few states based on the equations for kinematic model, so that car can plan the optimal trajectory for moving from one state to another.

Following are the state variables:
 1. __px__ : position in x-axis in map coordinates
 2. __py__ : position in y-axis in map coordinates
 3. __psi__: current orientation
 4. __v__: current velocity

These state variables are provided by the simulator along with the waypoints(in map coordinates) which are used as reference points where the vehicle should be in ideal situation. These waypoints are used to generate reference trajectory by fitting to a `3^rd` degree polynomial.


## Timestep Length and Elapsed Duration (N & dt)
## Polynomial Fitting and MPC Preprocessing
## Model Predictive Control with Latency
