#pragma once

#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include "RobotSim.hpp"

class Pendulum : public RobotInterface {
 public:
  Pendulum();
  virtual ~Pendulum();

  void step(state_t &resulting_state, state_t &state, control_t &u);
  void get_current_state(state_t &state);
  state_t reset_random();

 private:
  webots::Motor *pendulum_motor;
  webots::PositionSensor *pos_sensor;
  webots::PositionSensor *hip_sensor;
};
