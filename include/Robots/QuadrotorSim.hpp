#pragma once

#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include "RobotSim.hpp"

class QuadrotorSim : public RobotInterface {
 public:
  QuadrotorSim();
  virtual ~QuadrotorSim();

  void step(state_t &resulting_state, state_t &state, control_t &u);
  void get_current_state(state_t &state);
  state_t reset_random();

 private:
  webots::Supervisor *robot;
  webots::Node *robot_node;
  webots::InertialUnit *imu;
  webots::Gyro *gyro;
  webots::Compass *compass;
  webots::Accelerometer *accelerometer;
  webots::Camera *cam;

  std::array<webots::Motor *, 4> motors;
};
