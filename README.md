# Model Predictive Control Project

https://youtu.be/NGlVQ3fEfiE

In this project, I have implemented a Model Predictive Controller for controlling the steering and throttle of the car in simulator, such that it closely follows a simulator generated trajectory.


## Setup
Dependencies:
 1. cmake >= 3.5
 2. make >= 4.1
 3. gcc/g++ >= 5.4
 4. uWebSockets: run `./install-ubuntu.sh`.
 5. Ipopt and CppAD: Refer <a href="https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md">this document</a> for instructions.
 
 This project involves the Term 2 Simulator which can be downloaded <a href="https://github.com/udacity/self-driving-car-sim/releases">here</a>.
 
 To build the project run the following commands after cloning this repository:
  1. `cd build`
  2. `sudo chmod +x run.sh`
  3. `./run`

For more info, check <a href="https://github.com/udacity/CarND-MPC-Project">this repository</a>.

## The Model
In MPC we predict the next few states based on the equations for kinematic model, so that car can plan the optimal trajectory for moving from one state to another.

### State
Following are the state variables:
 1. __px__ : position in x-axis in map coordinates
 2. __py__ : position in y-axis in map coordinates
 3. __psi__: current orientation
 4. __v__ &nbsp; &nbsp;: current velocity

These state variables are provided by the simulator along with the waypoints(in map coordinates) which are used as reference points where the vehicle should be in ideal situation. These waypoints are used to generate reference trajectory by fitting to a 3rd degree polynomial.

### Error
Cross track error and error in orientation can be calculated by using the current and desired states.

__cte__ : Cross track error is difference between current and desired position. Here we can use our fitted polynomial coefficients to get cte when x=0 for current position(since vehicle is at center), which is `coeff[0]`(coeff is the fitted polynomial).

__epsi__: Error in orientation is the error between current and desired heading directions. It is equivalent to arctan of first order derivative of cte at x=0, which is `coeff[1]`.

### Controls
Here we have two control values output by the model which control the vehicle's behaviour,i.e, steering and throttle. Following are the variables used for the same.

__delta__: This is steering value. It is constrained to be in between -25 and 25 degrees converted to radians.

__a__ &nbsp; &nbsp; &nbsp;: This is throttle value. It is constrained to be in between -1 and 1. Negative values represents breaking and positive value for acceleration.

### Kinematic Model equations

Following are the equations used to get the next state after time elapsed `dt`:

* n_px = px + v\*cos(phi)\*dt
* n_px = py + v\*sin(phi)\*dt
* n_psi = psi + v\*(delta/Lf)*dt
* n_v = v + a\*dt
* n_cte = cte + v\*sin(epsi)\*dt;
where Lf is the distance between the front axle and center of mass of the vehicle.


## Timestep Length and Elapsed Duration (N & dt)
## Polynomial Fitting and MPC Preprocessing
## Model Predictive Control with Latency
