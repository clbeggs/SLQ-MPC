
#include <math.h>

#include <Dynamics.hpp>
#include <Eigen/Core>
#include <LQR.hpp>
#include <MujocoInterface.hpp>
#include <Robots.hpp>
#include <SLQ.hpp>
#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
using std::vector;

void run_slq_cartpole() {
  std::cout << "SLQ! Cartpole" << std::endl;
  state_t goal_state = state_t::Zero();
  goal_state << 0.0, 0.0, M_PI, 0.0;

  state_matrix_t A = state_matrix_t::Zero();
  control_gain_matrix_t B = control_gain_matrix_t::Zero();
  control_t u;
  u << 0.0;

  CartPole sys;
  RobotInterface *sim = new Pendulum();

  sys.linearize_dynamics(A, B, goal_state, u);
  control_feedback_t K = control_feedback_t::Zero();
  state_matrix_t P = state_matrix_t::Zero();
  state_matrix_t Q = state_matrix_t::Identity() * 10.0;
  Q.row(2)(2) = 20.0;
  Q.row(3)(3) = 20.0;

  control_matrix_t R = control_matrix_t::Zero();
  R(0) = 20.0;
  std::cout << R << std::endl;
  state_matrix_t W = state_matrix_t::Zero();

  state_t initial_state = sim->reset_random();
  sim->get_current_state(initial_state);
  std::cout << "Initial State: \n" << initial_state << std::endl;
  state_t xx;
  sim->get_current_state(xx);
  std::cout << "State: \n" << xx << std::endl;
  xx(1) = 0.174533;
  sim->set_state(xx);
  xx = sim->get_current_state();
  SLQ slq(sim, xx);
  sys.linearize_dynamics(A, B, xx, u);

  std::cout << "State: \n" << xx << std::endl;
  state_t next;
  for (int i = 0; i < 1000; ++i) {
    bool result = slq.compute(Q, R, A, B, K, P);
    std::cout << result << std::endl;
    if (!result) {
      K = control_feedback_t::Zero();
    }
    ////////////////////////////
    u = (-1) * K * xx;
    ////////////////////////////
    if (u(0) > 40) {
      u(0) = 40;
    } else if (u(0) < -40) {
      u(0) = -40;
    }
    std::cout << u << std::endl;
    std::cout << "X: " << std::endl;
    std::cout << sim->get_current_state() << std::endl;
    std::cout << "=-==================" << std::endl;
    xx = sim->step(xx, u);
    sys.linearize_dynamics(A, B, xx, u);
  }

  // std::cout << "LQR: " << result << std::endl;

  // std::cout << "K: \n" << K << std::endl;
  // vector<control_t> U(10, control_t::Zero());
  // state_t X = initial_state;

  // if (result) {
  // for (int i = 0; i < 10; i++) {
  // U[i] = K * X;
  // X = sys.forward_dynamics(X, U[i]);
  // if (X.norm() > 1e6 || U[i].norm() > 1e6) {
  // std::cout << "!!!!!!!!!!!!!!!!!!!!!1ERROR, X OR U GREATER THAN "
  // "1e6!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  // << std::endl;
  // std::cout << X << std::endl;
  // std::cout << U[i] << std::endl;
  // }
  // }
  // } else {
  // for (int i = 0; i < 10; i++) {
  // U[i] = control_t::Random();
  // }
  // }
  // for (int i = 0; i < 1000; ++i) {
  // initial_state = slq.solve_slq(Q, R, W, P, U, initial_state, goal_state);
  // }
}

int main() {
  run_slq_cartpole();
}
