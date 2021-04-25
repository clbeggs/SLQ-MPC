#pragma once

#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

class RobotInterface {
 public:
  virtual ~RobotInterface() {}
  virtual void step(state_t &resulting_state, state_t &state, control_t &u) {}
  virtual void get_current_state(state_t &state) {}
  state_t reset_random() {}

 protected:
  webots::Supervisor *robot;
  webots::Node *robot_node;
  int current_time;
};
