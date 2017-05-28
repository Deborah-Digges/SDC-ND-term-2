//============================================================================
// Name        : global-kinematic-model.cpp
// Author      : ddigges
// Version     :
// Copyright   : SDCND coursework
// Description : Hello World in C++, Ansi-style
//============================================================================

// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];

  double x_new = x + v * dt * cos(psi);
  double y_new = y + v * dt * sin(psi);
  double psi_new = psi + (v * dt)/Lf * actuators[0];
  double v_new = v + actuators[1] * dt;

  next_state << x_new, y_new, psi_new, v_new;

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}
