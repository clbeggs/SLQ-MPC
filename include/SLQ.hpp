#ifndef SLQ_H
#define SLQ_H

#include <Dynamics.hpp>
#include <Eigen/Core>
#include <LQR.hpp>
#include <Sim.hpp>
#include <types.hpp>
#include <vector>

using std::vector;

// clang-format off
class SLQ : public LQR {
 public:
  SLQ();
  SLQ(Sim_Interface *sim, state_t initial_state);
  ~SLQ();

  // Main algo driver
  void solve_slq(state_matrix_t &Q,
                 control_matrix_t &R,
                 state_matrix_t &W,
                 state_matrix_t &H,
                 vector<control_t> &initial_control, 
                 state_t &x_0,
                 state_t &x_g);


  vector<forward_t> _forward_rollout(state_t &initial_state,
                                     vector<control_t> &initial_control, 
                                     state_matrix_t &Q,
                                     control_matrix_t &R,
                                     state_matrix_t &H,
                                     state_t &goal_state);

  vector<backward_t> _backward_pass(vector<forward_t> forward_result);

  void second_expansion(forward_t &f,
                        state_t &V_x,
                        state_matrix_t &V_xx,
                        Q_t *q);

  fit_t fit(state_matrix_t &Q,
            control_matrix_t &R,
            state_matrix_t &W,
            state_matrix_t &H,
            vector<control_t> &initial_control, 
            state_t &x_0,
            state_t &x_g);

 protected:

  // Utility functions
  state_t _get_current_state();

  control_t _get_current_action();

  bool _check_converged(vector<lqr_t> &initial_cost, vector<lqr_t> &new_cost);

  LQR lqr;
  Sim_Interface *simulator;
  Quadrotor sys;
  float _delta = 1.0;
  float _delta_2 = 1.0;
  float _mu = 1.0;
  float _mu_min = 0.2;

  int max_loop_iters = 100;
  int max_line_search= 5;
  float alpha_d = 0.2;  // TODO: UPDATE
  float alpha = 0.0;

  state_t _current_state;
  control_t _current_action;
};

// clang-format on

#endif  // SLQ_H
