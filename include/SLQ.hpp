#ifndef SLQ_H
#define SLQ_H

#include <Dynamics.hpp>
#include <Eigen/Core>
#include <LQR.hpp>
#include <MujocoInterface.hpp>
#include <Robots.hpp>
#include <Sim.hpp>
#include <array>
#include <types.hpp>
#include <vector>
using std::array;
using std::vector;

// clang-format off
class SLQ : public LQR {
 public:
  SLQ();
  SLQ(RobotInterface *sim, state_t &initial_state);
  SLQ(RobotInterface *sim, MujocoSimulator *M, state_t &initial_state);
  ~SLQ();

  // Main algo driver
  state_t solve_slq(state_matrix_t &Q,
                 control_matrix_t &R,
                 state_matrix_t &W,
                 state_matrix_t &H,
                 vector<control_t> &initial_control, 
                 state_t &x_0,
                 state_t &x_g);


  void update_control(vector<control_t> &U,
                      trajectory_t &traj,
                      trajectory_t &old_traj,
                      vector<lqr_t> &ric,
                      state_matrix_t &Q,
                      control_matrix_t &R, 
                      state_matrix_t &P,
                      state_t &x_0,
                      float &J_0);

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
  RobotInterface *simulator;
  MujocoSimulator *M;
  CartPole sys;
  float _delta = 1.0;
  float _delta_2 = 1.0;
  float _mu = 1.0;
  float _mu_min = 0.2;

  int max_loop_iters = 100;
  int line_search_iters = 5;
  float alpha_d = 0.2;  // TODO: UPDATE
  float alpha = 0.1;
  float alpha_0 = 0.1;
  // clang-format off
  array<double, 20> pows = {0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0,
                            1.0, 1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0, 1.0, 1.0

  };
  array<double, 10> alphas = {1.0,
                              0.9090909090909091,
                              0.6830134553650705,
                              0.4240976183724846,
                              0.2176291357901485,
                              0.09229599817706402,
                              0.03234918430760664,
                              0.009370406407450668,
                              0.002243200793301555,
                              0.00044380531822726623};

  state_t _current_state;
  control_t _current_action;
};

#endif  // SLQ_H
