#include <math.h>

#include <Eigen/Core>
#include <cartpole.hpp>
#include <iostream>
#include <types.hpp>

CartPole::CartPole() {
  mass_cart = 1.05;
  mass_pole = 0.05;
  g = 9.80665;
  pole_length = 0.6;
  h = 0.002;
  N_c = 1.0;
}

CartPole::~CartPole() {
}

state_t CartPole::runge_kutta_step(state_t &x, control_t &u) {
  _k1 = forward_dynamics(x, u);

  _tmp = x + ((_k1 * h) / 2.0);
  _k2 = forward_dynamics(_tmp, u);

  _tmp = x + ((_k2 * h) / 2.0);
  _k3 = forward_dynamics(_tmp, u);

  _tmp = x + _k3;
  _k4 = forward_dynamics(_tmp, u);
  return (x + (h / 6.0) * (_k1 + (2 * _k2) + (2 * _k3) + _k4));
}
trajectory_t CartPole::runge_kutta(state_t &x0, vector<control_t> &U) {
  int N = U.size();
  trajectory_t traj(N, x0);

  // vector of dx, need to integrate!
  for (int i = 0; i < N - 1; ++i) {
    traj.x[i + 1] = runge_kutta_step(traj.x[i], U[i]);
  }
  return traj;
}

// state_t CartPole::forward_dynamics_friction(state_t &x_t, control_t &u_t) {
// double mu = 0.02;  // from webots world file

// state_t dx;
// double p_x = x_t(0);
// double theta = x_t(1);
// double dp_x = x_t(2);
// double dtheta = x_t(3);

// double M = mass_pole + mass_cart;
// double dtheta2 = pow(dtheta, 2.0);
// double m_p = mass_pole;
// double l = pole_length;
// double stheta = sin(theta);
// double ctheta = cos(theta);
// double c2theta = pow(ctheta, 2.0);

// dx(0) = dp_x;
// dx(1) = dtheta;

// double F = u_t(0);

// // clang-format off
// double a = ((-1.0) * F - (m_p * l * dtheta2 * (stheta + mu * N_c * ctheta)))
// / M; double num = (g * stheta) + (ctheta * (a + (mu * g * N_c))) - ((mu *
// dtheta) / (m_p * l));

// // clang-format on
// }

state_t CartPole::forward_dynamics(state_t &x_t, control_t &u_t) {
  state_t dx;
  double p_x = x_t(0);
  double theta = x_t(1);
  double dp_x = x_t(2);
  double dtheta = x_t(3);

  double M = mass_pole + mass_cart;
  double dtheta2 = pow(dtheta, 2.0);
  double m_p = mass_pole;
  double l = pole_length;
  double stheta = sin(theta);
  double ctheta = cos(theta);
  double c2theta = pow(ctheta, 2.0);

  dx(0) = dp_x;
  dx(1) = dtheta;

  double F = u_t(0);
  double a = (((-1.0) * F) - (m_p * l * dtheta2 * stheta)) / M;
  double denom = l * ((4.0 / 3.0) - ((m_p * c2theta) / M));

  // clang-format off
  dx(3) = ((g * stheta) + (ctheta * a)) / denom;

  a = m_p * l * (dtheta2 * stheta - dx(3) * ctheta);
  dx(2) = (F + a) / M;
  // clang-format on

  return dx;
}

trajectory_t CartPole::simulate_rollout(vector<control_t> &u, state_t &x0) {
  return runge_kutta(x0, u);
}

state_matrix_t CartPole::f_x(state_t &x_t, control_t &u_t) {
  state_matrix_t A = state_matrix_t::Zero();
  A.row(0)(0) = 0.0;
  A.row(0)(1) = 0.0;
  A.row(0)(2) = 1.0;
  A.row(0)(3) = 0.0;

  A.row(1)(0) = 0.0;
  A.row(1)(1) = 0.0;
  A.row(1)(2) = 0.0;
  A.row(1)(3) = 1.0;

  double p_x = x_t(0);
  double theta = x_t(1);
  double dp_x = x_t(2);
  double dtheta = x_t(3);
  double M = mass_pole + mass_cart;

  double ctheta = cos(theta);
  double c2theta = pow(cos(theta), 2.0);
  double stheta = sin(theta);
  double len_mp_dth = pole_length * pow(dtheta, 2.0) * mass_pole;
  double F = u_t(0);
  double m_p = mass_pole;
  double l = pole_length;
  double dtheta2 = pow(dtheta, 2.0);

  // clang-format off
  double a = (4.0 * M - 3.0 * m_p * c2theta);
  double b = ((-1.0) * stheta * ((-1.0) * l * m_p * dtheta2 * stheta - F));
  double c = l * m_p * dtheta2 * c2theta;
  double d = g * M * ctheta;
  double e = 18.0 * m_p * ctheta * stheta;
  double f = (-1.0) * l * m_p * dtheta2 * stheta - F;
  double h = g * M * stheta;
  double num = 3.0 * (a) * (b - c + d) - e * (ctheta * f + h);
  double denom = (4.0 * M - 3.0 * m_p * c2theta);
  denom = l * pow(denom, 2.0);

  A.row(2)(0) = 0.0;
  A.row(2)(1) = num / denom;
  A.row(2)(2) = 0.0;
  A.row(2)(3) = (2.0 * m_p * ctheta * stheta * dtheta) / (M * ((4.0 / 3.0) - ((m_p * c2theta) / M)));


  a = ((-1.0) * F - (m_p * l * dtheta2 * stheta));
  denom = l * ((4.0/3.0) - ((m_p * c2theta) / M));
  num = ((g * stheta) + (ctheta * (a / M)));
  double ddtheta = num / denom;

  A.row(3)(0) = 0.0;
  A.row(3)(1) = (l * m_p * ((ddtheta * stheta) + (dtheta2 * ctheta))) / M;
  A.row(3)(2) = 0.0;
  A.row(3)(3) = (2.0 * l * m_p * stheta * dtheta) / M;
  // clang-format on
  return A;
}

control_gain_matrix_t CartPole::f_u(state_t &x_t, control_t &u_t) {
  double theta = x_t(1);
  double ctheta = cos(theta);
  double c2theta = pow(cos(theta), 2.0);
  double M = mass_pole + mass_cart;
  double l = pole_length;
  double m_p = mass_pole;
  control_gain_matrix_t B = control_gain_matrix_t::Zero();
  // clang-format off

  B(2) = 1.0 / M;
  B(3) = ((-1.0) * ctheta) / (l * M * ((4.0 / 3.0) - ((m_p * c2theta) / M)));

  // clang-format on
  return B;
}

void CartPole::linearize_dynamics(state_matrix_t &A, control_gain_matrix_t &B,
                                  state_t &x_t, control_t &u_t) {
  A = f_x(x_t, u_t);
  B = f_u(x_t, u_t);
}

void CartPole::linearize_trajectory(trajectory_t &traj) {
  int N = traj.x.size();
  for (int i = 0; i < N; ++i) {
    linearize_dynamics(traj.A[i], traj.B[i], traj.x[i], traj.u[i]);
  }
}
