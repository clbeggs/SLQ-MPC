#ifndef LQR_H
#define LQR_H

#include <CARE.hpp>
#include <CostFunc.hpp>
#include <Dynamics.hpp>
#include <Eigen/Core>
#include <types.hpp>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>

using std::vector;

class LQR : public CARE {
 public:
  LQR();
  LQR(state_t goal_state);
  ~LQR();
  /*!
     Q - intermediate cost
     R - input cost
     K - kalman gain
  */
  bool compute(state_matrix_t &Q, control_matrix_t &R, state_matrix_t &A,
               control_gain_matrix_t &B, control_feedback_t &K,
               state_matrix_t &P);

  vector<lqr_t> riccati_like(trajectory_t &traj, state_matrix_t &Q,
                             control_matrix_t &R);

  CARE care;
  Quadrotor quadrotor_system;
  CostFunction cost_fn;
  state_t goal_state;
};

#endif  // LQR_H
