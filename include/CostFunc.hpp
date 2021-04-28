#ifndef COSTFUNC_H
#define COSTFUNC_H

#include <Eigen/Core>
#include <types.hpp>
#include <vector>

class CostFunction {
 public:
  CostFunction() {
    this->_stable_state = state_t::Ones();
    this->_hover = control_t::Zero();
    this->goal_state = state_t::Zero();
    this->goal_action = control_t::Zero();
  }

  CostFunction(state_t &goal_state, control_t &goal_action) : CostFunction() {
    this->goal_state = goal_state;
    this->goal_action = goal_action;
  }
  ~CostFunction() {}

  // clang-format off
  float traj_cost(std::vector<state_t> &x,
                  std::vector<control_t> &u,
                  state_matrix_t &Q,
                  control_matrix_t &R,
                  state_matrix_t &H);

  void change_goal(state_t &goal_state, control_t &goal_action) { 
    this->goal_state = goal_state;
    this->goal_action = goal_action;
  }

  float l(state_t &x,
          control_t &u,
          state_matrix_t &Q,
          control_matrix_t &R,
          bool terminal=false);

  state_t l_x(state_t &x,
              control_t &u,
              state_matrix_t &Q,
              control_matrix_t &R,
              bool terminal=false);

  control_t l_u(state_t &x,
                control_t &u,
                state_matrix_t &Q,
                control_matrix_t &R,
                bool terminal=false);

  state_matrix_t l_xx(state_t &x,
                      control_t &u,
                      state_matrix_t &Q,
                      control_matrix_t &R,
                      bool terminal=false);

  control_matrix_t l_uu(state_t &x,
                        control_t &u,
                        state_matrix_t &Q,
                        control_matrix_t &R,
                        bool terminal=false);

  float quadratize_trajectory_cost(trajectory_t &traj,
                                   state_matrix_t &Q,
                                   control_matrix_t &R,
                                   state_matrix_t &P_tf);


  void get_cost(state_matrix_t &Q,
                control_matrix_t &R,
                state_t &x,
                control_t &u,
                cost_t &cst);

  float objective_function(state_matrix_t &H,
                           state_matrix_t &Q,
                           control_matrix_t &R,
                           state_matrix_t &W,
                           std::vector<state_t> &x,
                           std::vector<control_t> &u);
  // clang-format on

  cost_t get_terminal_cost(state_matrix_t &H, state_t x, cost_t &cst);

  state_t _stable_state;
  control_t _hover;

  state_t goal_state;
  control_t goal_action;
};

#endif  // COSTFUNC_H
