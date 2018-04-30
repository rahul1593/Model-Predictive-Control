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

ctet+1​=ctet​+vt​∗sin(eψt​)∗dt
## Timestep Length and Elapsed Duration (N & dt)
## Polynomial Fitting and MPC Preprocessing
## Model Predictive Control with Latency
