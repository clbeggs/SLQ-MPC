
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
  goal_state << 5.0, 0.0, 0.0, 0.0;

  state_matrix_t A;
  control_gain_matrix_t B;
  control_t u;
  u << 0.0;

  CartPole sys;
  RobotInterface *sim = new Pendulum();

  sys.linearize_dynamics(A, B, goal_state, u);
  control_feedback_t K;
  state_matrix_t P;
  state_matrix_t Q = state_matrix_t::Zero();
  Q.row(0)(0) = 500.0;
  Q.row(1)(1) = 100.0;

  control_matrix_t R = control_matrix_t::Zero();
  R(0) = 1;
  state_matrix_t W = state_matrix_t::Zero();

  state_t initial_state = sim->reset_random();
  sim->get_current_state(initial_state);
  std::cout << "Initial State: \n" << initial_state << std::endl;

  SLQ slq(sim, initial_state);

  bool result = slq.compute(Q, R, A, B, K, P);

  std::cout << "K: \n" << K << std::endl;
  vector<control_t> U(10);
  state_t X = initial_state;

  for (int i = 0; i < 9; i++) {
    U[i] = K * X;
    X = sys.forward_dynamics(X, U[i]);
    if (X.norm() > 1e6 || U[i].norm() > 1e6) {
      std::cout << "!!!!!!!!!!!!!!!!!!!!!1ERROR, X OR U GREATER THAN "
                   "1e6!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                << std::endl;
      std::cout << X << std::endl;
      std::cout << U[i] << std::endl;
    }
  }

  slq.solve_slq(Q, R, W, P, U, initial_state, goal_state);
}

int main() {
  run_slq_cartpole();
}
