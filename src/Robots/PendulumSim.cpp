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
  robot->step(current_time);
}

Pendulum::~Pendulum() {
}

void Pendulum::step(state_t &resulting_state, state_t &x_t, control_t &u_t) {
  this->pendulum_motor->setForce(u_t(0));
  robot->step(current_time);
  get_current_state(resulting_state);
}

void Pendulum::get_current_state(state_t &x) {
  const double *pos = robot_node->getPosition();
  const double *vel = robot_node->getVelocity();
  // TODO: Check position and velocity fields

  x(0) = pos[0];
  x(1) = hip_sensor->getValue();
  x(2) = vel[0];
  x(3) = vel[3];
}

state_t Pendulum::reset_random() {
  const webots::Supervisor::SimulationMode mode = robot->simulationGetMode();
  robot->simulationSetMode(
      webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);

  robot->simulationResetPhysics();
  robot->simulationReset();
  robot_node->resetPhysics();

  robot->simulationSetMode(mode);
  state_t x;
  get_current_state(x);
  return x;
}
