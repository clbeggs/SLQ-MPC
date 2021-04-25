#include <math.h>

#include <Eigen/Core>
#include <LQR.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>

using std::vector;

LQR::LQR() {
}
LQR::LQR(state_t &x_g) {
  this->goal_state = x_g;
}
LQR::~LQR() {
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

  bool success = solve_care(Q, R, A, B, P, R_inverse, true);

  K = (R_inverse * (B.transpose() * P));

  return success;
}

vector<lqr_t> LQR::riccati_like(trajectory_t &traj, state_matrix_t &Q,
                                control_matrix_t &R) {
  int N = traj.x.size();
  vector<lqr_t> out(N);

  compute(Q, R, traj.A[N - 1], traj.B[N - 1], out[N - 1].K, out[N - 1].P);
  out[N - 1].p = state_t::Zero();

  control_matrix_t H;
  control_t g;
  control_feedback_t G;

  for (int i = N - 2; i > 0; i--) {
    // Compute Ricatti
    compute(Q, R, traj.A[i], traj.B[i], out[i].K, out[i].P);

    // Fill in vals for iLQR
    H = R + traj.B[i].transpose() * out[i + 1].P * traj.B[i];
    out[i].q = cost_fn.l_x(traj.x[i], traj.u[i], Q, R);

    out[i].r = cost_fn.l_u(traj.x[i], traj.u[i], Q, R);

    g = out[i].r + (traj.B[i].transpose() * out[i + 1].p);

    out[i].l_t = -H.inverse() * g;

    out[i].p = out[i].q + (traj.A[i].transpose() * out[i + 1].p) +
               (out[i].K.transpose() * H * out[i].l_t) +
               (out[i].l_t.transpose() * g * state_t::Ones()) +
               (G.transpose() * out[i].l_t);
  }

  return out;
}
