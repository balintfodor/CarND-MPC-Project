#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> solx;
  vector<double> soly;

  static const double steering_limit_rad;
};

Eigen::VectorXd globalKinematicStep(double x0, double y0,
  double psi0, double v0, double dt, double delta, double a);

#endif /* MPC_H */
