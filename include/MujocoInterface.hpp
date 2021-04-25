#include <mjxmacro.h>
#include <mujoco.h>
#include <stdio.h>
#include <stdlib.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <types.hpp>

#pragma once

class MujocoSimulator {
 public:
  MujocoSimulator();
  MujocoSimulator(char* model_file);
  ~MujocoSimulator();

  state_matrix_t calc_A(state_t& x, control_t& u);
  control_gain_matrix_t calc_B(state_t& x, control_t& u);
  void linearize_trajectory(trajectory_t& traj);

  trajectory_t mujoco_rollout(std::vector<control_t>& u,
                              state_t& initial_state);

  state_t mujoco_forward_dyn(state_t& x, control_t& u);

  void compute_jacobians();
  double _perturb(state_t& x, state_t& x1);

 private:
  void _restore_state();
  void _save_state();
  void _change_state(state_t& x0);
  state_t _get_state();
  void _actuate_motors(control_t& u);

  mjData* model_state;
  mjModel* model;
  // Saving state vars:
  mjtNum time;
  std::vector<mjtNum> qpos, qvel, act;
  int nq, nv, na;

  // Derivative variables
  double time_step_size = 0.002;

  // NQ = 7
  // NV = 6
};
