#ifndef SIM_H
#define SIM_H

#include <EigenRand/EigenRand>
#include <array>
#include <types.hpp>
#include <vector>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
class Sim_Interface {
 public:
  Sim_Interface();
  ~Sim_Interface();

  void step(state_t &resulting_state, state_t &state, control_t &action);
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
  webots::Motor *pendulum_motor;
  webots::PositionSensor *pos_sensor;
  webots::PositionSensor *hip_sensor;

  std::array<webots::Motor *, 4> motors;
  int current_time;
};

#endif  // SIM_H
