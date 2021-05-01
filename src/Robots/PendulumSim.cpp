#include <PendulumSim.hpp>
#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

Pendulum::Pendulum() {
  robot = new webots::Supervisor();
  robot_node = robot->getSelf();
  if (robot_node == NULL) {
    std::cout << "Robot Node IS NULL!" << std::endl;
  }

  pendulum_motor = robot->getMotor("horizontal_motor");
  pos_sensor = robot->getPositionSensor("horizontal position sensor");
  hip_sensor = robot->getPositionSensor("hip");
  current_time = (int)robot->getBasicTimeStep();

  pos_sensor->enable(current_time);
  hip_sensor->enable(current_time);
  trans_field = robot_node->getField("translation");
  hip = robot->getFromDef("HIP");
  pole = robot->getFromDef("THIGH_BB");
  cart_base = robot->getFromDef("CART_BASE");
  webots::Field *hip_joint_field = hip->getField("jointParameters");
  webots::Node *hip_pos_node = hip_joint_field->getSFNode();
  hip_joint = hip_pos_node->getField("position");
  robot->step(current_time);
}

Pendulum::~Pendulum() {
}

void Pendulum::step(state_t &resulting_state, state_t &x_t, control_t &u_t) {
  this->pendulum_motor->setForce(u_t(0));
  robot->step(current_time);
  // std::cout << "Time: " << robot->getTime() << std::endl;
  get_current_state(resulting_state);
}

state_t Pendulum::step(state_t &x_t, control_t &u_t) {
  this->pendulum_motor->setForce(u_t(0));
  robot->step(current_time);
  // std::cout << "Time: " << robot->getTime() << std::endl;
  return get_current_state();
}

void Pendulum::set_state(state_t &x) {
  const double *vals = cart_base->getPosition();
  const double POS[3] = {vals[0], vals[1], x(0)};
  trans_field->setSFVec3f(POS);
  hip_joint->setSFFloat(x(1));
  robot_node->resetPhysics();
  robot->step(current_time);
  robot->step(current_time);
}

void Pendulum::get_current_state(state_t &x) {
  x = get_current_state();
}

state_t Pendulum::get_current_state() {
  state_t x;
  const double *pos = cart_base->getPosition();
  const double *vel = cart_base->getVelocity();
  double hip_pos = hip_joint->getSFFloat();
  const double *pole_vel = pole->getVelocity();

  x(0) = pos[2];
  x(1) = hip_pos;
  x(2) = vel[2];
  x(3) = pole_vel[3];
  return x;
}
state_t Pendulum::reset_random() {
  const webots::Supervisor::SimulationMode mode = robot->simulationGetMode();
  robot->simulationSetMode(
      webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);

  robot->simulationResetPhysics();
  robot->simulationReset();
  robot_node->resetPhysics();

  robot->simulationSetMode(mode);
  state_t x = state_t::Zero();
  get_current_state(x);
  return x;
}
