#include <EigenRand/EigenRand>
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

Sim_Interface::Sim_Interface() {
  robot = new webots::Supervisor();
  robot_node = robot->getSelf();
  if (robot_node == NULL) {
    std::cout << "QUADROTOR IS NULL!" << std::endl;
  }

  // robot = new webots::Robot();
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

  webots::Field *tst = robot_node->getField("physics");
  webots::Field *tstp = robot_node->getProtoField("physics");
  if (tst) {
    printf("GET FIELD IS GOOD:\n");
  } else {
    printf("GET FIELD IS NOTTTTTt GOOD:\n");
  }
  if (tstp) {
    printf("GET PROTo FIELD IS GOOD:\n");

  } else {
    printf("GET pROToOOO FIELD IS NOTTTTTt GOOD:\n");
  }
}

Sim_Interface::~Sim_Interface() {
}

std::vector<state_t> Sim_Interface::rollout(
    std::vector<control_t> action_sequence, state_t x_0) {
  std::vector<state_t> state_sequence;
  state_sequence.push_back(x_0);  // TODO: Change with reserve()

  state_t cur_state = x_0;
  state_t next_state;

  for (int i = 0; i < action_sequence.size(); i++) {
    step(next_state, cur_state, action_sequence[i]);
    state_sequence.push_back(next_state);
    cur_state = next_state;
  }

  return state_sequence;
}

void Sim_Interface::get_current_state(state_t &state) {
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

void Sim_Interface::step(state_t &resulting_state, state_t &state,
                         control_t &action) {
  // Actuate motors
  motors[0]->setVelocity((double)action(0));
  motors[1]->setVelocity(-(double)action(1));
  motors[2]->setVelocity(-(double)action(2));
  motors[3]->setVelocity((double)action(3));

  // Step in sim
  robot->step(current_time);

  // angular positions
  // const double roll = imu->getRollPitchYaw()[0] + M_PI; // same as phi
  // const double pitch = imu->getRollPitchYaw()[1];       // same as
  // theta const double yaw = imu->getRollPitchYaw()[2];         // same
  // as psi

  // angular velocities
  const double roll_velocity = gyro->getValues()[0];
  const double pitch_velocity = gyro->getValues()[1];
  const double yaw_velocity = gyro->getValues()[2];

  // position accelerations
  const double x_accel = accelerometer->getValues()[0];
  const double y_accel = accelerometer->getValues()[1];
  const double z_accel = accelerometer->getValues()[2];

  // position velocity from angular velocity
  //
  // https::physics.stackexchange.com/questions/53843/3d-get-linear-velocity-from-position-and-angular-velocity

  // position velocities

  // TODO: Derivative approximation, from current and past angular
  // velocities
  // angular accelerations
  webots::Field *trans_field = robot_node->getField("translation");
  const double *pos = trans_field->getSFVec3f();
  const double *vel = robot_node->getVelocity();
  const double *pos2 = robot_node->getPosition();
  // Populate state vector
  // resulting_state(0, 2) = 0;
  resulting_state(0) = (float)pos[0];
  resulting_state(1) = (float)pos[1];
  resulting_state(2) = (float)pos[2];
  resulting_state(3) = (float)roll_velocity;
  resulting_state(4) = (float)pitch_velocity;
  resulting_state(5) = (float)yaw_velocity;
  resulting_state(6) = (float)x_accel;
  resulting_state(7) = (float)y_accel;
  resulting_state(8) = (float)z_accel;
}

state_t Sim_Interface::get_initial_state() {
  state_t x;
  webots::Field *trans_field = robot_node->getField("translation");
  webots::Field *rot_field = robot_node->getField("rotation");
  const double *pos = trans_field->getSFVec3f();
  const double *rot = rot_field->getSFRotation();
}

state_t Sim_Interface::reset_random() {
  const webots::Supervisor::SimulationMode mode = robot->simulationGetMode();
  robot->simulationSetMode(
      webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
  // Eigen::Rand::Vmt19937_64 urng{42};

  // Eigen::Vector3f mean = Eigen::Vector3f::Ones();
  // Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();

  // constructs MvNormalGen with Scalar=float, Dim=4
  // Eigen::Rand::MvNormalGen<float, 3> gen1{mean, cov};
  // Eigen::Vector3f new_pos = gen1.generate(urng);
  Eigen::Vector3f new_pos =
      Eigen::Vector3f::Random() + Eigen::Vector3f::Ones() * 2;

  webots::Field *trans_field = robot_node->getField("translation");
  webots::Field *rot_field = robot_node->getField("rotation");
  const double rot_vals[4] = {-0.024659, -0.706892, -0.706891, -3.0923};
  rot_field->setSFRotation(rot_vals);

  // const double vals[3] = {new_pos(0), new_pos(1), new_pos(2)};
  const double vals[3] = {1.0, 1.0, 1.0};
  const double *old = trans_field->getSFVec3f();

  printf("New Position: (%f, %f, %f)\n", vals[0], vals[1], vals[2]);
  printf("Old Position: (%f, %f, %f)\n", old[0], old[1], old[2]);

  robot->simulationResetPhysics();
  robot->simulationReset();

  trans_field->setSFVec3f(vals);
  robot_node->resetPhysics();

  state_t pos = state_t::Zero();
  pos(0) = vals[0];
  pos(1) = vals[1];
  pos(2) = vals[2];

  pos(3) = -1.5707981;
  pos(4) = 0.0697283;
  pos(5) = -3.1415817;

  robot->simulationSetMode(mode);
  return pos;
}
void Sim_Interface::reset_random(state_t state, control_t action) {
}
