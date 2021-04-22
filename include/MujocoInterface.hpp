#include <mjxmacro.h>
#include <mujoco.h>
#include <stdio.h>
#include <stdlib.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <types.hpp>

class MujocoSimulator {
 public:
  MujocoSimulator();
  MujocoSimulator(char* model_file);
  ~MujocoSimulator();

  trajectory_t mujoco_rollout(std::vector<control_t>& u,
                              state_t& initial_state);

  state_t mujoco_forward_dyn(state_t& x, control_t& u);

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
};
