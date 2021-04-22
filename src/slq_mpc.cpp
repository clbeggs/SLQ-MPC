
#include <Dynamics.hpp>
#include <Eigen/Core>
#include <LQR.hpp>
#include <MujocoInterface.hpp>
#include <SLQ.hpp>
#include <Sim.hpp>
#include <iostream>
#include <types.hpp>
#include <vector>
using std::vector;

void run_slq() {
  std::cout << "SLQ!" << std::endl;
  state_t goal_state = state_t::Zero();
  goal_state(0) = 2.0;
  goal_state(1) = 2.0;
  goal_state(2) = 2.0;
  goal_state(3) = 0.0004869;
  goal_state(4) = 0.0382203;
  goal_state(5) = 0.0382018;
  state_matrix_t A;
  control_gain_matrix_t B;
  control_t u;
  u << 69.0, 69.0, 69.0, 69.0;

  Quadrotor sys;
  Sim_Interface sim;
  // clang-format off
  MujocoSimulator M("/home/epiphyte/Documents/Homework/NumericalAnalysis/final/models/quadrotor_ground.xml");
  // clang-format on

  sys.linearize_dynamics(A, B, goal_state, u);
  control_feedback_t K;
  state_matrix_t P;
  state_matrix_t Q = state_matrix_t::Identity() * 2;
  Q(0, 0) = Q(1, 1) = Q(2, 2) = 100.0;  // position
  Q(6, 6) = Q(7, 7) = Q(8, 8) = 10.0;   // linear velocity
  control_matrix_t R = control_matrix_t::Identity() * 0.5;
  state_matrix_t W = state_matrix_t::Zero();

  state_t initial_state = sim.reset_random();

  SLQ slq(&sim, initial_state);

  bool result = slq.compute(Q, R, A, B, K, P);

  vector<control_t> U(10);
  state_t X;
  for (int i = 0; i < 9; i++) {
    U[i] = K * X;
    X = M.mujoco_forward_dyn(X, U[i]);
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
  run_slq();
}
