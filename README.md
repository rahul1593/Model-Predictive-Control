# Model Predictive Control Project

In this project, I have implemented a Model Predictive Controller for controlling the steering and throttle of the car in simulator, such that it closely follows a simulator generated trajectory.

Following is the output video for this project.
https://youtu.be/NGlVQ3fEfiE

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

## Model Predictive Control
In MPC we predict the next few states based on the equations for kinematic model, so that the vehicle can plan the optimal trajectory for moving from one state to another.

MPC uses an optimizer to minimise the cost associate with the actuations according to the given cost function.

Following is the flow of code for this project:
* Get the current state from the simulator
* Generate the reference trajectory and claculate the cross-track and orientation errors.
* Predict the state 100ms in the future to account for delay in actuations and pass the state to the solver.
* Solver uses the cost function to get the cost for the predictions it makes and tries to minimise the cost for the returned output.
* The state returned by the solver is send to the simulator with 100ms delay and the process is repeated.

### State
Following are the state variables:
 * __px__ : position in x-axis in map coordinates
 * __py__ : position in y-axis in map coordinates
 * __psi__: current orientation
 * __v__ &nbsp; &nbsp;: current velocity

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
* n_epsi = epsi +  v\*(delta/Lf)*dt

where Lf is the distance between the front axle and center of mass of the vehicle.

### Cost Function
The cost funtion determines how much should we penalise the different errors such that the optimal predictions result in accurate and smooth actions by the vehicle. Following are the errors accounted for in the cost function:

* Cross track error
* Error in orientation
* Error in velocity
* Error in steering angle to acoid unnecessary steerinf
* Error in acceleration to avoid unnecessary throttle
* Difference between subsequent steering angles to avoid sudden steering
* Difference in subsequent throttle values to avoid sudden acceleration and braking

Following are the weights used for each of the above errors respectively:
* cte_w = 115.1
* epsi_w = 95.1
* v_w = 0.007
* dlt_w = 95.1
* a_w = 1.1
* dltd_w = 20.1
* ad_w = 1.1

## Timestep Length and Elapsed Duration (N & dt)
`N` is the length of timesteps for which we predict state where each timestep is of duration `dt`. 1 second is the most practical value for lookahead since the world is dynamic and sudden changes might result in change of course, so looking beyond 1 second may be useless. So `N*dt` should be approximately 1 second. Using `dt=0.1` (along with `n=10`) seemed too short duration (experimentally) for reacting, since it mostly destabilized the vehicle. So, I used `dt=0.15` and `N=8` to get smooth transit. Also we have 100 millisecond latency. So the choosen values result in optimal output.

## Polynomial Fitting and MPC Preprocessing
For getting the reference trajectory, the waypoints provided by the simulator need to be converted to the vehicle coordinate system. The converted waypoints are then used to fit a 3rd degree polynomial. Cross track error and orientation error is also calculated using the polynomial coefficients.

Following code snippet show the code for the same:
```C
          const int points_n = ptsx.size();
          Eigen::VectorXd vpts_x(points_n);
          Eigen::VectorXd vpts_y(points_n);
          
          for(int i = 0; i < points_n; ++i) {
            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;
            
            vpts_x[i] = dx * cos(-psi) - dy * sin(-psi);
            vpts_y[i] = dy * cos(-psi) + dx * sin(-psi);
          }
          
          // fit polynomial
          auto pf = polyfit(vpts_x, vpts_y, 3);
          
          // generate error estimates
          double cte = pf[0];

          // heading error
          double epsi = -atan(pf[1]);
 ```

## Model Predictive Control with Latency
To mimic the real world conditions, the latency of 100 milliseconds is introduced in the repsonse to the simulator. To account for this latency the model must predict the actuations for state after 100 ms in future.

Following code snippet shows the calculation of state after 100ms, which is then used for the actuation prediction:
```C
          //delay in actuator response
          const double dt = 0.1;
          
          // Vehicle is assumed to be moving along x-axis and vehicle is located at the center of it
          // so, {px, py, psi = 0.0, 0.0, 0.0}
          px = 0.0 + v * dt;
          py = 0.0;
          v = v + a * dt;
          psi = 0.0 + v * (delta) / Lf * dt;
          cte = cte + v * sin(epsi) * dt;
          epsi = epsi + v * (delta / Lf) * dt;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          // solve to get the next state using MPC
          auto res = mpc.Solve(state, pf);
          steer_value = -res[6];
          throttle_value = res[7];
```
