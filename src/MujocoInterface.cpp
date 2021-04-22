#include <mjxmacro.h>
#include <mujoco.h>
#include <stdio.h>
#include <stdlib.h>

#include <MujocoInterface.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <types.hpp>

MujocoSimulator::MujocoSimulator() {
}

MujocoSimulator::MujocoSimulator(char* model_file) {
  mj_activate("/home/epiphyte/.mujoco/mjkey.txt");
  char error[1000] = "Could not load binary model";
  this->model = mj_loadXML(model_file, 0, error, 1000);

  if (!this->model) {
    std::cout << "ERROR LOADING MUJOCO MODEL" << std::endl;
    std::cout << error << std::endl;
    throw -1;
  }
  this->model_state = mj_makeData(this->model);

  if (!this->model_state) {
    std::cout << "COULD NOT ALLOCAT MjDATA" << std::endl;
    throw -1;
  }
  this->nq = this->model->nq;
  this->nv = this->model->nv;
  this->na = this->model->na;
  this->qpos.reserve(this->nq);
  this->qvel.reserve(this->nv);
  this->act.reserve(this->na);
}

MujocoSimulator::~MujocoSimulator() {
  if (this->model_state) {
    mj_deleteData(this->model_state);
  }
  // if (this->model_state_copy) {
  // mj_deleteData(this->model_state_copy);
  // }
  if (this->model) {
    mj_deleteModel(this->model);
  }
  mj_deactivate();
}

void MujocoSimulator::_restore_state() {
  this->model_state->time = this->time;

  for (int i = 0; i < this->nq; ++i) {
    this->model_state->qpos[i] = this->qpos[i];
  }
  for (int i = 0; i < this->nv; ++i) {
    this->model_state->qvel[i] = this->qvel[i];
  }
  for (int i = 0; i < this->na; ++i) {
    this->model_state->act[i] = this->act[i];
  }
}

void MujocoSimulator::_save_state() {
  this->time = this->model_state->time;
  for (int i = 0; i < this->nq; ++i) {
    this->qpos[i] = this->model_state->qpos[i];
  }
  for (int i = 0; i < this->nv; ++i) {
    this->qvel[i] = this->model_state->qvel[i];
  }
  for (int i = 0; i < this->na; ++i) {
    this->act[i] = this->model_state->act[i];
  }
}

void MujocoSimulator::_change_state(state_t& x) {
  this->model_state->time = 1;
  // nq = 7
  // nv = 6

  this->model_state->qpos[0] = (double)x(0);
  this->model_state->qpos[1] = (double)x(1);
  this->model_state->qpos[2] = (double)x(2);

  float qx, qy, qz, qw, angle;
  // rotation of X is AXIS-ANGLE * NORM, i.e. rotation vector
  angle = sqrt(pow(x(3), 2) + pow(x(4), 2) + pow(x(5), 2));
  qx = (x(3) / angle) * sin(angle / 2);
  qy = (x(4) / angle) * sin(angle / 2);
  qz = (x(5) / angle) * sin(angle / 2);
  qw = cos(angle / 2);
  this->model_state->qpos[3] = (double)qw;
  this->model_state->qpos[4] = (double)qx;
  this->model_state->qpos[5] = (double)qy;
  this->model_state->qpos[6] = (double)qz;

  this->model_state->qvel[0] = (double)x(6);
  this->model_state->qvel[1] = (double)x(7);
  this->model_state->qvel[2] = (double)x(8);

  this->model_state->qvel[3] = (double)x(9);
  this->model_state->qvel[4] = (double)x(10);
  this->model_state->qvel[5] = (double)x(11);

  for (int i = 0; i < this->na; ++i) {
    this->model_state->act[i] = 0;
  }
}

void MujocoSimulator::_actuate_motors(control_t& u) {
  double thrust_1, thrust_2, thrust_3, thrust_4;
  thrust_1 = (0.00026) * abs(u(0)) * u(0);
  thrust_2 = (-0.00026) * abs(u(1)) * u(1);
  thrust_3 = (-0.00026) * abs(u(2)) * u(2);
  thrust_4 = (0.00026) * abs(u(3)) * u(3);
  std::cout << "THRUST: " << (0.00026) * abs(u(3)) * u(3) << std::endl;
  this->model_state->act[0] = thrust_1;
  this->model_state->act[1] = thrust_2;
  this->model_state->act[2] = thrust_3;
  this->model_state->act[3] = thrust_4;
}

state_t MujocoSimulator::_get_state() {
  state_t x;

  std::cout << "GET STATE" << std::endl;
  std::cout << "GET STATE" << std::endl;
  std::cout << "GET STATE" << std::endl;
  std::cout << "GET STATE" << std::endl;
  std::cout << this->model_state->qpos[0] << std::endl;
  std::cout << this->model_state->qpos[1] << std::endl;
  std::cout << this->model_state->qpos[2] << std::endl;

  std::cout << "GET STATE float" << std::endl;
  std::cout << (float)this->model_state->qpos[0] << std::endl;
  std::cout << (float)this->model_state->qpos[1] << std::endl;
  std::cout << (float)this->model_state->qpos[2] << std::endl;

  x(0) = (float)this->model_state->qpos[0];
  x(1) = (float)this->model_state->qpos[1];
  x(2) = (float)this->model_state->qpos[2];
  float qx, qy, qz, qw, angle;
  qx = this->model_state->qpos[3];
  qy = this->model_state->qpos[4];
  qz = this->model_state->qpos[5];
  qw = this->model_state->qpos[6];

  angle = 2 * acos(qw);
  x(3) = (angle * qx) / sqrt(1 - qw * qw);
  x(4) = (angle * qy) / sqrt(1 - qw * qw);
  x(5) = (angle * qz) / sqrt(1 - qw * qw);

  x(6) = (float)this->model_state->qvel[0];
  x(7) = (float)this->model_state->qvel[1];
  x(8) = (float)this->model_state->qvel[2];
  x(9) = (float)this->model_state->qvel[3];
  x(10) = (float)this->model_state->qvel[4];
  x(11) = (float)this->model_state->qvel[5];
  return x;
}

state_t MujocoSimulator::mujoco_forward_dyn(state_t& x, control_t& u) {
  _change_state(x);
  // Fill in all quantaties
  mj_forward(this->model, this->model_state);

  _actuate_motors(u);
  mj_step(this->model, this->model_state);
  return _get_state();
}

trajectory_t MujocoSimulator::mujoco_rollout(std::vector<control_t>& u,
                                             state_t& initial_state) {
  _change_state(initial_state);
  // Fill in all quantaties
  mj_forward(this->model, this->model_state);

  printf("MODEL QPOS: \n");
  std::cout << this->model_state->qpos[0] << std::endl;
  std::cout << this->model_state->qpos[1] << std::endl;
  std::cout << this->model_state->qpos[2] << std::endl;

  int N = u.size();
  printf("U SIZE: \n");
  std::cout << u.size() << std::endl;

  trajectory_t traj(N, initial_state);
  for (int i = 0; i < u.size() - 1; i++) {
    _actuate_motors(u[i]);
    mj_step(this->model, this->model_state);
    traj.x[i + 1] = _get_state();
    traj.u[i] = u[i];
  }

  return traj;
}
