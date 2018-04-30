#include "MPC.h"
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// Set the timestep length and duration, more duration is required for higher speeds
size_t N = 8;
double dt = 0.15;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//reference velocity
double ref_v = 80.0;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

//set as supplied through command line, else as below
double cte_w = 115.0;
double epsi_w = 95.0;
double v_w = 0.007;
double dlt_w = 95.0;
double a_w = 1.0;
double dltd_w = 20.0;
double ad_w = 1.0;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    //initialize cost to zero
    fg[0] = 0.0;
    size_t t;
    
    // The part of the cost based on the reference state.
    for (t = 0; t < N; t++) {
      fg[0] += cte_w * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_w * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_w * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += dlt_w * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_w * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += dltd_w * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += ad_w * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * x0 * x0) + (coeffs[3] * x0 * x0 * x0);
      // atan(df/dx)
      AD<double> psides0 = CppAD::atan( coeffs[1] + (2 * x0 *coeffs[2]) + (3 * x0 * x0 * coeffs[3]) );

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * (delta0 / Lf) * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * (delta0 / Lf) * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  // Initialize frequent components in constructor
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t i;
  
  this->n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  this->n_constraints = N * 6;
  
  this->vars_lowerbound = Dvector(this->n_vars);
  this->vars_upperbound = Dvector(this->n_vars);
  // TODO: Set lower and upper limits for variables.
  
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < delta_start; i++) {
    this->vars_lowerbound[i] = -1.0e19;
    this->vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (i = delta_start; i < a_start; i++) {
    this->vars_lowerbound[i] = -0.43;
    this->vars_upperbound[i] = 0.43;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (i = a_start; i < this->n_vars; i++) {
    this->vars_lowerbound[i] = -1.0;
    this->vars_upperbound[i] = 1.0;
  }
  
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  this->vars = Dvector(this->n_vars);
  for (i = 0; i < this->n_vars; i++) {
    this->vars[i] = 0.0;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  this->constraints_lowerbound = Dvector(this->n_constraints);
  this->constraints_upperbound = Dvector(this->n_constraints);
  for (i = 0; i < this->n_constraints; i++) {
    this->constraints_lowerbound[i] = 0.0;
    this->constraints_upperbound[i] = 0.0;
  }
}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Set the initial variable values
  this->vars[x_start] = x;
  this->vars[y_start] = y;
  this->vars[psi_start] = psi;
  this->vars[v_start] = v;
  this->vars[cte_start] = cte;
  this->vars[epsi_start] = epsi;
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.(Done in constructor)
  this->constraints_lowerbound[x_start] = x;
  this->constraints_lowerbound[y_start] = y;
  this->constraints_lowerbound[psi_start] = psi;
  this->constraints_lowerbound[v_start] = v;
  this->constraints_lowerbound[cte_start] = cte;
  this->constraints_lowerbound[epsi_start] = epsi;

  this->constraints_upperbound[x_start] = x;
  this->constraints_upperbound[y_start] = y;
  this->constraints_upperbound[psi_start] = psi;
  this->constraints_upperbound[v_start] = v;
  this->constraints_upperbound[cte_start] = cte;
  this->constraints_upperbound[epsi_start] = epsi;
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, this->vars, this->vars_lowerbound, this->vars_upperbound, this->constraints_lowerbound,
      this->constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  this->mpc_x_vals = {};
  this->mpc_y_vals = {};
  // store mpc trajectory
  for(i=0; i<N; i++){
    this->mpc_x_vals.push_back(solution.x[x_start + i]);
    this->mpc_y_vals.push_back(solution.x[y_start + i]);
  }
  
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start], solution.x[a_start]};
}
