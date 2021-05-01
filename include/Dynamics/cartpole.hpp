

#pragma once
#include <Eigen/Core>
#include <types.hpp>
#include <vector>

using std::vector;
class CartPole {
 public:
  CartPole();

  ~CartPole();

  state_t forward_dynamics(state_t &x_t, control_t &u_t);

  trajectory_t simulate_rollout(vector<control_t> &u, state_t &x0);

  void linearize_dynamics(state_matrix_t &A, control_gain_matrix_t &B,
                          state_t &x_t, control_t &u_t);

  void linearize_trajectory(trajectory_t &traj);
  state_matrix_t f_x(state_t &x, control_t &u);
  control_gain_matrix_t f_u(state_t &x, control_t &u);

  trajectory_t runge_kutta(state_t &x0, vector<control_t> &U);
  state_t runge_kutta_step(state_t &x, control_t &u);

 private:
  float mass_pole, mass_cart, pole_length, g;

  double N_c;
  state_t _k1, _k2, _k3, _k4, _tmp;
  double h;
};
