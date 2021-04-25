#ifndef QUADROTOR_TYPES_H
#define QUADROTOR_TYPES_H

#include <Eigen/Core>
#include <vector>
using std::vector;

typedef Eigen::Matrix<float, 4, 1> state_t;
typedef Eigen::Matrix<float, 1, 1> control_t;
typedef Eigen::Matrix<float, 4, 4> state_matrix_t;
typedef Eigen::Matrix<float, 1, 1> control_matrix_t;
typedef Eigen::Matrix<float, 4, 1> control_gain_matrix_t;
typedef Eigen::Matrix<float, 1, 4> control_feedback_t;

struct lqr_t {
  lqr_t() { l_t = control_t::Zero(); }
  lqr_t(float val) { l_t = control_t::Ones() * val; }
  control_t l_t;
  control_feedback_t K;
  state_matrix_t P;
  state_t p;
  control_t r;
  state_t q;
};

struct cost_t {
  float J;
  state_t l_x;
  control_t l_t;
  state_t p;
};

// struct lqr_t {
// control_feedback_t K;
// state_matrix_t P;
// };

struct forward_t {
  float l;
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
