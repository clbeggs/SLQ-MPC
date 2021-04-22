#include <CostFunc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// clang-format off
float CostFunction::l(state_t &x,
                      control_t &u,
                      state_matrix_t &Q,
                      control_matrix_t &R,
                      bool terminal) {
  // clang-format on
  if (terminal) {
    return ((x - this->goal_state).transpose() * Q * (x - this->goal_state))(0);
  } else {
    return ((x - this->goal_state).transpose() * Q *
            (x - this->goal_state))(0) +
           ((u - this->goal_action).transpose() * R *
            (u - this->goal_action))(0);
  }
}

// clang-format off
state_t CostFunction::l_x(state_t &x,
                          control_t &u,
                          state_matrix_t &Q,
                          control_matrix_t &R,
                          bool terminal) {
  // clang-format on
  return (x - this->goal_state).transpose() * (Q + Q.transpose());
}

// clang-format off
control_t CostFunction::l_u(state_t &x,
                            control_t &u,
                            state_matrix_t &Q,
                            control_matrix_t &R,
                            bool terminal) {
  // clang-format on
  if (terminal) {
    return control_t::Zero();
  } else {
    return (u - this->goal_action).transpose() * (R + R.transpose());
  }
}

// clang-format off
  state_matrix_t CostFunction::l_xx(state_t &x,
                                    control_t &u,
                                    state_matrix_t &Q,
                                    control_matrix_t &R,
                                    bool terminal){
  // clang-format on
  return Q + Q.transpose();
}

// clang-format off
  control_matrix_t CostFunction::l_uu(state_t &x,
                                      control_t &u,
                                      state_matrix_t &Q,
                                      control_matrix_t &R,
                                      bool terminal){
  // clang-format on
  if (terminal) {
    return control_matrix_t::Zero();
  } else {
    return R + R.transpose();
  }
}

// clang-format off
float CostFunction::objective_function(state_matrix_t &H,
                                       state_matrix_t &Q,
                                       control_matrix_t &R,
                                       state_matrix_t &W,
                                       std::vector<state_t> &x,
                                       std::vector<control_t> &u) {
  // clang-format on
  int N = x.size();
  float cost = (x[N - 1].transpose() * H * x[N - 1])(0);

  for (int i = 0; i < N; i++) {
    cost += (x[i].transpose() * Q * x[i])(0) + (u[i].transpose() * R * u[i])(0);
  }
  return cost;
}

void CostFunction::get_cost(state_matrix_t &Q, control_matrix_t &R, state_t &x,
                            control_t &u, cost_t &cst) {
  cst.J = (x.transpose() * Q * x)(0) + (u.transpose() * R * u)(0);
  cst.l_x = Q * (x - this->_stable_state);
}

cost_t CostFunction::get_terminal_cost(state_matrix_t &H, state_t x,
                                       cost_t &cst) {
  cst.J = (x.transpose() * H * x)(0);
  cst.l_x = H * (x - this->_stable_state);
  cst.p = cst.l_x;
  return cst;
}

float CostFunction::quadratize_trajectory_cost(trajectory_t &traj,
                                               state_matrix_t &Q,
                                               control_matrix_t &R,
                                               state_matrix_t &P_tf) {
  int N = traj.x.size();
  // x_tf = traj.x[N-1];
  float J = (traj.x[N - 1].transpose() * P_tf * traj.x[N - 1])(0);

  for (int i = 0; i < N; i++) {
    J += (traj.x[i].transpose() * Q * traj.x[i])(0) +
         (traj.u[i].transpose() * R * traj.u[i])(0);
  }
  return J;
}

float CostFunction::traj_cost(std::vector<state_t> &x,
                              std::vector<control_t> &u, state_matrix_t &Q,
                              control_matrix_t &R, state_matrix_t &H) {
  int N = x.size();
  float J = (x[N - 1].transpose() * H * x[N - 1])(0);
  for (int i = 0; i < N; i++) {
    J += l(x[i], u[i], Q, R);
  }
  return J;
}
