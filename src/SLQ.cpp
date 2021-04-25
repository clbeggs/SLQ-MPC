#include <math.h>

#include <Eigen/Core>
#include <LQR.hpp>
#include <MujocoInterface.hpp>
#include <SLQ.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>

using std::vector;

SLQ::SLQ() {
}

SLQ::~SLQ() {
}

SLQ::SLQ(RobotInterface *sim, state_t &initial_state) {
  this->simulator = sim;
  this->_current_state = initial_state;
  this->_current_action = control_t::Ones();
}

SLQ::SLQ(RobotInterface *sim, MujocoSimulator *M, state_t &initial_state) {
  this->simulator = sim;
  this->_current_state = initial_state;
  this->_current_action = control_t::Ones();
  this->M = M;
}

///////////////////////
// Utility functions //
///////////////////////
state_t SLQ::_get_current_state() {
  return this->_current_state;
}

control_t SLQ::_get_current_action() {
  return this->_current_action;
}

bool SLQ::_check_converged(vector<lqr_t> &initial_cost,
                           vector<lqr_t> &new_cost) {
  int N = initial_cost.size();
  for (int i = 0; i < N; i++) {
    if ((initial_cost[i].l_t(0) <= new_cost[i].l_t(0)) &&
        (initial_cost[i].l_t(1) <= new_cost[i].l_t(1)) &&
        (initial_cost[i].l_t(2) <= new_cost[i].l_t(2)) &&
        (initial_cost[i].l_t(3) <= new_cost[i].l_t(3))) {
      return false;
    }
  }
  return true;
}

//////////////////////////
// Main Driver Function //
//////////////////////////

// clang-format off
vector<forward_t> SLQ::_forward_rollout(state_t &initial_state,
                                        vector<control_t> &u,
                                        state_matrix_t &Q,
                                        control_matrix_t &R,
                                        state_matrix_t &H,
                                        state_t &goal_state) {
  // clang-format on
  int N = u.size();
  vector<forward_t> result(N);

  result[0].x = initial_state;

  for (int i = 0; i < N - 1; i++) {
    result[i + 1].x = sys.forward_dynamics(result[i].x, u[i]);
    // result[i + 1].x = this->M->mujoco_forward_dyn(result[i].x, u[i]);
    result[i].l = cost_fn.l(result[i].x, u[i], Q, R, false);
    result[i].l_x = cost_fn.l_x(result[i].x, u[i], Q, R, false);

    result[i].l_u = cost_fn.l_u(result[i].x, u[i], Q, R, false);
    result[i].l_xx = cost_fn.l_xx(result[i].x, u[i], Q, R, false);
    result[i].l_uu = cost_fn.l_uu(result[i].x, u[i], Q, R, false);

    sys.linearize_dynamics(result[i].A, result[i].B, result[i].x, u[i]);
  }

  result[N - 1].l = cost_fn.l(result[N - 1].x, u[0], Q, R, true);
  result[N - 1].l_x = cost_fn.l_x(result[N - 1].x, u[0], Q, R, true);
  result[N - 1].l_xx = cost_fn.l_xx(result[N - 1].x, u[0], Q, R, true);

  return result;
}

vector<backward_t> SLQ::_backward_pass(vector<forward_t> forward_result) {
  int N = forward_result.size();
  vector<Q_t> QQ(N);
  vector<backward_t> BB(N);
  state_matrix_t V_xx = forward_result[N - 1].l_xx;
  state_matrix_t V_xx_tmp;
  state_t V_x = forward_result[N - 1].l_x;

  for (int i = 0; i < N; i++) {
    second_expansion(forward_result[i], V_x, V_xx, &QQ[i]);
    BB[i].k = -QQ[i].Q_uu.colPivHouseholderQr().solve(QQ[i].Q_u);
    BB[i].K = -QQ[i].Q_uu.colPivHouseholderQr().solve(QQ[i].Q_ux);
    if (isnan(BB[i].k(0))) {
      BB[i].k = control_t::Zero();
    }
    if (isnan(BB[i].K(0))) {
      BB[i].K = control_feedback_t::Zero();
    }

    V_x = (QQ[i].Q_x + BB[i].K.transpose() * QQ[i].Q_uu * BB[i].k) +
          (BB[i].K.transpose() * QQ[i].Q_u + QQ[i].Q_ux.transpose() * BB[i].k);

    V_xx_tmp = (QQ[i].Q_xx + BB[i].K.transpose() * QQ[i].Q_uu * BB[i].K) +
               (BB[i].K.transpose() * QQ[i].Q_ux) +
               (QQ[i].Q_ux.transpose() * BB[i].K);

    V_xx = 0.5 * (V_xx_tmp + V_xx_tmp.transpose());
  }
  return BB;
}

void SLQ::second_expansion(forward_t &f_out, state_t &V_x, state_matrix_t &V_xx,
                           Q_t *q) {
  q->Q_x = f_out.l_x + (f_out.A.transpose() * V_x);
  q->Q_u = f_out.l_u + (f_out.B.transpose() * V_x);
  q->Q_xx = f_out.l_xx + (f_out.A.transpose() * V_xx * f_out.A);

  state_matrix_t reg = this->_mu * state_matrix_t::Identity();
  q->Q_ux = f_out.B.transpose() * (V_xx + reg) * f_out.A;
  q->Q_uu = f_out.B.transpose() * (V_xx + reg) * f_out.B;
}

// clang-format off
void SLQ::solve_slq(state_matrix_t &Q,
                    control_matrix_t &R,
                    state_matrix_t &W,
                    state_matrix_t &P_tf,
                    vector<control_t> &u, 
                    state_t &x_0, 
                    state_t &x_g) {
  // clang-format on

  // Simulate system dynamics
  trajectory_t old_traj = sys.simulate_rollout(u, x_0);
  // trajectory_t old_traj = M->mujoco_rollout(u, x_0);

  // Linearize dynamics along trajectory
  sys.linearize_trajectory(old_traj);

  // Quadratize cost along trajectory
  float J_0 = cost_fn.quadratize_trajectory_cost(old_traj, Q, R, P_tf);
  float J_new = J_0 + 1;

  int main_loop_iters = 0;
  while (this->max_loop_iters > main_loop_iters) {
    // Simulate system dynamics
    trajectory_t traj = sys.simulate_rollout(u, x_0);
    // trajectory_t traj = M->mujoco_rollout(u, x_0);

    // Linearize dynamics along trajectory
    sys.linearize_trajectory(traj);

    // Backwards solve Ricatti-like diff. eqn.'s
    vector<lqr_t> ric = riccati_like(traj, Q, R);

    // Line search to update control
    int N = traj.x.size();
    for (int i = 0; i < N; i++) {
      u[i] = u[i] + this->alpha * ric[i].l_t +
             ric[i].K * (traj.x[i] - old_traj.x[i]);
    }

    J_new = cost_fn.quadratize_trajectory_cost(traj, Q, R, P_tf);
    if (J_new < J_0) {
      break;
    }
    main_loop_iters += 1;
  }
}
