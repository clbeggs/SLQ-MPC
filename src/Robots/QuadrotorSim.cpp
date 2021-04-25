#include <QuadrotorSim.hpp>
#include <RobotSim.hpp>
#include <Sim.hpp>
#include <iostream>
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
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

QuadrotorSim::QuadrotorSim() {
  robot = new webots::Supervisor();
  robot_node = robot->getSelf();
  if (robot_node == NULL) {
    std::cout << "Robot Node IS NULL!" << std::endl;
  }

  // Init sensors
  imu = robot->getInertialUnit("inertial unit");
  gyro = robot->getGyro("gyro");
  compass = robot->getCompass("compass");
  accelerometer = robot->getAccelerometer("accelerometer");
  cam = robot->getCamera("camera");

  // Init motors
  motors[0] = robot->getMotor("front left propeller");
  motors[1] = robot->getMotor("front right propeller");
  motors[2] = robot->getMotor("rear left propeller");
  motors[3] = robot->getMotor("rear right propeller");

  // Enable sensors
  current_time = (int)robot->getBasicTimeStep();

  imu->enable(current_time);
  accelerometer->enable(current_time);
  gyro->enable(current_time);
  compass->enable(current_time);
  cam->enable(current_time);

  motors[0]->setPosition(INFINITY);
  motors[1]->setPosition(INFINITY);
  motors[2]->setPosition(INFINITY);
  motors[3]->setPosition(INFINITY);
  motors[0]->setVelocity(1.0);
  motors[1]->setVelocity(-1.0);
  motors[2]->setVelocity(-1.0);
  motors[3]->setVelocity(1.0);
  robot->step(current_time);
}

QuadrotorSim::~QuadrotorSim() {
}

void QuadrotorSim::get_current_state(state_t &state) {
  const double *pos = robot_node->getPosition();
  const double *rot = robot_node->getOrientation();
  const double *vel = robot_node->getVelocity();

  double qh = sqrt(1 + rot[0] + rot[4] + rot[8]) / 2;
  double qx = (rot[7] - rot[5]) / (4 * qh);
  double qy = (rot[2] - rot[6]) / (4 * qh);
  double qz = (rot[3] - rot[1]) / (4 * qh);

  state(0) = (float)pos[0];
  state(1) = (float)pos[1];
  state(2) = (float)pos[2];

  state(3) = (float)(atan2(2 * (qh * qx + qy * qz),
                           1 - 2 * (pow(qx, 2) + pow(qy, 2))));
  state(4) = (float)asin(2 * (qh * qy - qz * qx));
  state(5) = (float)(atan2(2 * (qh * qz + qx * qy),
                           1 - 2 * (pow(qy, 2) + pow(qz, 2))));

  state(6) = (float)vel[0];
  state(7) = (float)vel[1];
  state(8) = (float)vel[2];

  state(9) = (float)vel[3];
  state(10) = (float)vel[4];
  state(11) = (float)vel[5];
}

void QuadrotorSim::step(state_t &resulting_state, state_t &state,
                        control_t &action) {
  // Actuate motors
  motors[0]->setVelocity((double)action(0));
  motors[1]->setVelocity(-(double)action(1));
  motors[2]->setVelocity(-(double)action(2));
  motors[3]->setVelocity((double)action(3));

  // Step in sim
  robot->step(current_time);

  get_current_state(resulting_state);
}

state_t QuadrotorSim::reset_random() {
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
