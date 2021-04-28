#include <math.h>

#include <Eigen/Core>
#include <LQR.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>

using std::vector;

LQR::LQR() {
  cost_fn.goal_action = control_t::Zero();
}
LQR::LQR(state_t &x_g) {
  this->goal_state = x_g;
  cost_fn.goal_state = x_g;
  cost_fn.goal_action = control_t::Zero();
}

LQR::~LQR() {
}

void LQR::change_goal(state_t &goal_state) {
  this->goal_state = goal_state;
  cost_fn.goal_state = goal_state;
  cost_fn.goal_action = control_t::Zero();
}
// clang-format off
bool LQR::compute(state_matrix_t &Q,
                  control_matrix_t &R,
                  state_matrix_t &A,
                  control_gain_matrix_t &B,
                  control_feedback_t &K,
                  state_matrix_t &P) {
  // clang-format on
  control_matrix_t R_inverse;

  bool success = solve_care(Q, R, A, B, P, R_inverse, false);

  K = (R_inverse * (B.transpose() * P));

  return success;
}

vector<lqr_t> LQR::riccati_like(trajectory_t &traj, state_matrix_t &Q,
                                control_matrix_t &R) {
  int N = traj.x.size();
  vector<lqr_t> out(N);

  // compute(Q, R, traj.A[N - 1], traj.B[N - 1], out[N - 1].K, out[N - 1].P);
  out[N - 1].p = state_t::Zero();
  out[N - 1].P = state_matrix_t::Zero();

  control_matrix_t H;
  control_t g;
  control_feedback_t G = control_feedback_t::Zero();

  for (int i = N - 2; i > 0; i--) {
    // Compute Ricatti
    // compute(Q, R, traj.A[i], traj.B[i], out[i].K, out[i].P);

    // Fill in vals for iLQR
    H = R + traj.B[i].transpose() * out[i + 1].P * traj.B[i];
    G = traj.B[i].transpose() * out[i + 1].P * traj.A[i];
    out[i].K = -H.inverse() * G;

    out[i].P = Q + (traj.A[i].transpose() * out[i + 1].P * traj.A[i]) +
               (out[i].K.transpose() * H * out[i].K) +
               (out[i].K.transpose() * G) + (G.transpose() * out[i].K);

    out[i].q = cost_fn.l_x(traj.x[i], traj.u[i], Q, R);

    out[i].r = cost_fn.l_u(traj.x[i], traj.u[i], Q, R);

    g = out[i].r + (traj.B[i].transpose() * out[i + 1].p);

    out[i].l_t = -H.inverse() * g;

    std::cout << out[i].q << std::endl;
    std::cout << (traj.A[i].transpose() * out[i + 1].p) << std::endl;
    std::cout << (out[i].K.transpose() * H * out[i].l_t) << std::endl;
    std::cout << (out[i].l_t.transpose() * g * state_t::Ones()) << std::endl;
    std::cout << (G.transpose() * out[i].l_t) << std::endl;

    out[i].p = out[i].q + (traj.A[i].transpose() * out[i + 1].p) +
               (out[i].K.transpose() * H * out[i].l_t) +
               (out[i].l_t.transpose() * g * state_t::Ones()) +
               (G.transpose() * out[i].l_t);
  }

  return out;
}
