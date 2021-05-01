#ifndef QUADROTOR_TYPES_H
#define QUADROTOR_TYPES_H

#include <Eigen/Core>
#include <vector>
using std::vector;

// typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
// typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
// typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
// typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
// typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

// typedef Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> schur_matrix_t;
// typedef Eigen::Matrix<double, 2 * STATE_DIM, STATE_DIM> factor_matrix_t;

typedef Eigen::Matrix<double, 8, 8> schur_matrix_t;
typedef Eigen::Matrix<double, 8, 4> factor_matrix_t;

typedef Eigen::Matrix<double, 4, 1> state_t;
typedef Eigen::Matrix<double, 1, 1> control_t;
typedef Eigen::Matrix<double, 4, 4> state_matrix_t;
typedef Eigen::Matrix<double, 1, 1> control_matrix_t;
typedef Eigen::Matrix<double, 4, 1> control_gain_matrix_t;
typedef Eigen::Matrix<double, 1, 4> control_feedback_t;

struct lqr_t {
  lqr_t() {
    l_t = control_t::Zero();
    K = control_feedback_t::Zero();
    P = state_matrix_t::Zero();
    p = state_t::Zero();
    r = control_t::Zero();
    q = state_t::Zero();
  }
  lqr_t(double val) {
    l_t = control_t::Ones() * val;
    K = control_feedback_t::Zero();
    P = state_matrix_t::Zero();
    p = state_t::Zero();
    r = control_t::Zero();
    q = state_t::Zero();
  }
  control_t l_t;
  control_feedback_t K;
  state_matrix_t P;
  state_t p;
  control_t r;
  state_t q;
};

struct cost_t {
  cost_t() {
    J = 0.0;
    l_x = state_t::Zero();
    l_t = control_t::Zero();
    p = state_t::Zero();
  }

  double J;
  state_t l_x;
  control_t l_t;
  state_t p;
};

// struct lqr_t {
// control_feedback_t K;
// state_matrix_t P;
// };

struct forward_t {
  double l;
  control_t l_u;
  state_t l_x;
  state_matrix_t l_xx;
  control_matrix_t l_uu;

  state_matrix_t A;
  control_gain_matrix_t B;

  state_t x;
};

struct Q_t {
  state_matrix_t Q;
  state_t Q_x;
  control_t Q_u;
  state_matrix_t Q_xx;
  control_matrix_t Q_uu;
  control_feedback_t Q_ux;
};

struct backward_t {
  control_feedback_t K;
  control_t k;
};

struct fit_t {
  fit_t() {}
  fit_t(int N) : x(N), u(N) {}
  vector<state_t> x;
  vector<control_t> u;
  backward_t Ks;
};

struct trajectory_t {
  trajectory_t() {}
  trajectory_t(int N, state_t &x0)
      : u(N, control_t::Zero()),
        x(N, state_t::Zero()),
        A(N, state_matrix_t::Zero()),
        B(N, control_gain_matrix_t::Zero()) {
    x[0] = x0;
  }
  vector<state_t> x;
  vector<control_t> u;
  vector<state_matrix_t> A;
  vector<control_gain_matrix_t> B;
};

#endif  // QUADROTOR_TYPES_H
