

#include <math.h>

#include <Eigen/Core>
#include <cartpole.hpp>
#include <types.hpp>

CartPole::CartPole() {
}

CartPole::~CartPole() {
}

state_t CartPole::forward_dynamics(state_t &x_t, control_t &u_t) {
  state_t dx;
  float p_x = x_t(0);
  float theta = x_t(1);
  float dp_x = x_t(2);
  float dtheta = x_t(3);

  float M = mass_pole + mass_cart;
  dx(0) = dp_x;
  dx(1) = dtheta;

  float num =
      g * sin(theta) + cos(theta) *
                           ((-u_t(0) - mass_pole * pole_length *
                                           pow(dtheta, 2.0) * sin(theta))) /
                           M;
  dx(3) = num / (pole_length *
                 ((4.0 / 3.0) * (mass_pole * pow(cos(theta), 2.0)) / (M)));

  num = u_t(0) + mass_pole * pole_length *
                     (pow(dtheta, 2) * sin(theta) - dx(3) * cos(theta));
  dx(2) = num / M;
  return dx;
}

trajectory_t CartPole::simulate_rollout(vector<control_t> &u, state_t &x0) {
}

void CartPole::linearize_dynamics(state_matrix_t &A, control_gain_matrix_t &B,
                                  state_t &x_t, control_t &u_t) {
  A.row(0)(0) = 0;
  A.row(0)(1) = 0;
  A.row(0)(2) = 1;
  A.row(0)(3) = 0;

  A.row(1)(0) = 0;
  A.row(1)(1) = 0;
  A.row(1)(2) = 0;
  A.row(1)(3) = 1;

  float p_x = x_t(0);
  float theta = x_t(1);
  float dp_x = x_t(2);
  float dtheta = x_t(3);
  float M = mass_pole + mass_cart;

  float ctheta = cos(theta);
  float c2theta = pow(cos(theta), 2.0);
  float stheta = sin(theta);
  float len_mp_dth = pole_length * pow(dtheta, 2.0) * mass_pole;
  float F = u_t(0);

  float num = (-stheta * len_mp_dth * stheta - F);
  num += -len_mp_dth * c2theta + g * M * ctheta;
  num *= 3.0 * (4 * M - 3.0 * mass_pole * c2theta);
  num += -18.0 * mass_pole * ctheta * stheta *
         (ctheta * (len_mp_dth * stheta - F) + g * M * stheta);

  A.row(2)(1) =
      num / pow((pole_length * (4.0 * M - 3.0 * mass_pole * c2theta)), 2.0);
  A.row(2)(0) = 0.0;

  A.row(2)(2) = 0.0;
  A.row(2)(3) = (2 * mass_pole * ctheta * stheta * dtheta) /
                (M * ((4.0 / 3.0) - ((mass_pole * c2theta) / M)));

  A.row(3)(0) = 0.0;

  num = g * sin(theta) + cos(theta) *
                             ((-u_t(0) - mass_pole * pole_length *
                                             pow(dtheta, 2.0) * sin(theta))) /
                             M;
  float ddtheta =
      num /
      (pole_length * ((4.0 / 3.0) * (mass_pole * pow(cos(theta), 2.0)) / (M)));

  A.row(3)(1) = (pole_length * mass_pole *
                 (ddtheta * stheta + pow(dtheta, 2.0) * ctheta));
  A.row(3)(3) = (2 * pole_length * mass_pole * stheta * dtheta) / M;
  A.row(3)(0) = 0.0;
  A.row(3)(2) = 0.0;

  B(0) = 0.0;
  B(1) = (3 * ctheta) / (pole_length * (3 * mass_pole * c2theta - 4 * M));
}

void CartPole::linearize_trajectory(trajectory_t &traj) {
}
