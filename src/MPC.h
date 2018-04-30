#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using CppAD::AD;


extern size_t N;
extern const double Lf;

extern double cte_w;
extern double epsi_w;
extern double v_w;
extern double dlt_w;
extern double a_w;
extern double dltd_w;
extern double ad_w;

extern size_t x_start;
extern size_t y_start;
extern size_t psi_start;
extern size_t v_start;
extern size_t cte_start;
extern size_t epsi_start;
extern size_t delta_start;
extern size_t a_start;

typedef CPPAD_TESTVECTOR(double) Dvector;
  
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

double polyeval(Eigen::VectorXd coeffs, double x);

class MPC {
 public:
   
  size_t n_vars;
  size_t n_constraints;
  
  Dvector vars_lowerbound;
  Dvector vars_upperbound;
  Dvector vars;
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;
  
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;
  
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
