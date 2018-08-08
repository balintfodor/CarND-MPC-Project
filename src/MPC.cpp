#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "poly.h"

using CppAD::AD;

 const double MPC::steering_limit_rad = 0.436332313; // 25 deg

// TODO: Set the timestep length and duration
const size_t N = 20;
const double dt = 0.05;

const size_t n_state = 6;
const size_t n_act = 2;

const size_t start_x = 0;
const size_t start_y = N;
const size_t start_psi = 2 * N;
const size_t start_v = 3 * N;
const size_t start_cte = 4 * N;
const size_t start_epsi = 5 * N;

const size_t start_delta = 6 * N;
const size_t start_a = start_delta + N - 1;

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

Eigen::VectorXd globalKinematicStep(double x0, double y0,
  double psi0, double v0, double dt, double delta, double a)
{
  Eigen::VectorXd new_state(4);
  new_state(0) = x0 + v0 * cos(psi0) * dt;
  new_state(1) = y0 + v0 * sin(psi0) * dt;
  new_state(2) = psi0 - v0 / Lf * delta * dt;
  new_state(3) = v0 + a * dt;
  return new_state;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    static const double invLf = 1.0 / Lf;

    // initial values
    fg[1 + start_x] = vars[start_x];
    fg[1 + start_y] = vars[start_y];
    fg[1 + start_psi] = vars[start_psi];
    fg[1 + start_v] = vars[start_v];
    fg[1 + start_cte] = vars[start_cte];
    fg[1 + start_epsi] = vars[start_epsi];

    // constraints connecting timesteps
    for (int i = 1; i < N; ++i) {
      AD<double> x1 = vars[start_x + i];
      AD<double> y1 = vars[start_y + i];
      AD<double> psi1 = vars[start_psi + i];
      AD<double> v1 = vars[start_v + i];
      AD<double> cte1 = vars[start_cte + i];
      AD<double> epsi1 = vars[start_epsi + i];

      AD<double> x0 = vars[start_x + i - 1];
      AD<double> y0 = vars[start_y + i - 1];
      AD<double> psi0 = vars[start_psi + i - 1];
      AD<double> v0 = vars[start_v + i - 1];
      AD<double> cte0 = vars[start_cte + i - 1];
      AD<double> epsi0 = vars[start_epsi + i - 1];

      AD<double> delta0 = vars[start_delta + i - 1];
      AD<double> a0 = vars[start_a + i - 1];

      // kinematic equations
      fg[1 + start_x + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + start_y + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + start_psi + i] = psi1 - (psi0 - v0 * invLf * delta0 * dt);
      fg[1 + start_v + i] = v1 - (v0 + a0 * dt);
      fg[1 + start_cte + i] = cte1 - (y0 - polyeval(coeffs, x0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + start_epsi + i] = epsi1 - (epsi0 - v0 * invLf * delta0 * dt);
    }

    // cost function
    fg[0] = 0;

    for (int i = 0; i < N; ++i) {
      double w = (N - i) / (double)N;
      fg[0] += w * CppAD::pow(vars[start_cte + i], 2);
      fg[0] += w * CppAD::pow(vars[start_epsi + i], 2);
      fg[0] += w * (-200) * vars[start_v + i];
    }

    for (int i = 0; i < N - 1; ++i) {
      double w = (N - 1 - i) / (double)(N - 1);
      fg[0] += w * CppAD::pow(vars[start_v + i], 2) * CppAD::pow(vars[start_delta + i], 2);
      fg[0] += w * 100 * CppAD::pow(vars[start_a + i], 2);
      fg[0] += w * 10 * CppAD::pow(vars[start_cte + i + 1] - vars[start_cte + i], 2);
      fg[0] += w * 10 * CppAD::pow(vars[start_epsi + i + 1] - vars[start_epsi + i], 2);
    }

    for (int i = 0; i < N - 2; ++i) {
      double w = (N - 2 - i) / (double)(N - 2);
      fg[0] += w * 100 * CppAD::pow(vars[start_delta + i + 1] - vars[start_delta + i], 2);
      fg[0] += w * 100 * CppAD::pow(vars[start_a + i + 1] - vars[start_a + i], 2);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = n_state * N + n_act * (N - 1);
  // TODO: Set the number of constraints
  size_t n_constraints = n_state * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[start_x] = state(0);
  vars[start_y] = state(1);
  vars[start_psi] = state(2);
  vars[start_v] = state(3);
  vars[start_cte] = state(4);
  vars[start_epsi] = state(5);

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  for (int i = 0; i < start_delta; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = start_delta; i < start_a; i++) {
    vars_lowerbound[i] = -steering_limit_rad;
    vars_upperbound[i] = steering_limit_rad;
  }

  for (int i = start_a; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[start_x] = state(0);
  constraints_lowerbound[start_y] = state(1);
  constraints_lowerbound[start_psi] = state(2);
  constraints_lowerbound[start_v] = state(3);
  constraints_lowerbound[start_cte] = state(4);
  constraints_lowerbound[start_epsi] = state(5);

  constraints_upperbound[start_x] = state(0);
  constraints_upperbound[start_y] = state(1);
  constraints_upperbound[start_psi] = state(2);
  constraints_upperbound[start_v] = state(3);
  constraints_upperbound[start_cte] = state(4);
  constraints_upperbound[start_epsi] = state(5);

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
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  solx.clear();
  soly.clear();
  for (int i = 0; i < N; ++i) {
    solx.push_back(solution.x[start_x + i]);
    soly.push_back(solution.x[start_y + i]);
  }
  return {solution.x[start_delta], solution.x[start_a]};
}
