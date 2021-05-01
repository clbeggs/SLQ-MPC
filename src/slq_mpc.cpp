
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

void run_lqr() {
  std::cout << "LQR Cartpole=======" << std::endl;
  state_t goal_state = state_t::Zero();
  goal_state << 10.0, 0.0, 0.0, 0.0;

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
  state_matrix_t W = state_matrix_t::Zero();

  state_t initial_state = sim->reset_random();
  sim->get_current_state(initial_state);
  state_t xx;
  sim->get_current_state(xx);
  xx(1) = 0.174533;
  sim->set_state(xx);
  xx = sim->get_current_state();
  SLQ slq(sim, xx);
  sys.linearize_dynamics(A, B, xx, u);

  state_t next = xx;
  for (int i = 0; i < 1000; ++i) {
    bool result = slq.compute(Q, R, A, B, K, P);
    if (!result) {
      K = control_feedback_t::Zero();
    }
    ////////////////////////////
    u = (-1.0) * K * xx;
    ////////////////////////////
    if (u(0) > 40) {
      u(0) = 40;
    } else if (u(0) < -40) {
      u(0) = -40;
    }
    next = sys.runge_kutta_step(next, u);
    xx = sim->step(xx, u);
    std::cout << "Dist: " << (next - xx).squaredNorm() << std::endl;
    // std::cout << "X; " << next(0) << "ACtual: " << xx(0) << std::endl
    // << std::endl;
    printf("%f ___ %f\n", next(0), xx(0));
    printf("%f ___ %f\n", next(1), xx(1));
    printf("%f ___ %f\n", next(2), xx(2));
    printf("%f ___ %f\n", next(3), xx(3));

    sys.linearize_dynamics(A, B, xx, u);
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void run_slq_cartpole() {
  state_t goal_state = state_t::Zero();
  goal_state << 10.0, 0.0, 0.0, 0.0;

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
  state_matrix_t W = state_matrix_t::Zero();

  state_t x0;
  sim->get_current_state(x0);
  x0(1) = 0.174533;
  sim->set_state(x0);
  x0 = sim->get_current_state();
  SLQ slq(sim, x0);
  // P_tf
  sys.linearize_dynamics(A, B, goal_state, u);
  bool result = slq.compute(Q, R, A, B, K, P);
  state_t next = x0;

  std::vector<control_t> U;
  for (int i = 0; i < 200; ++i) {
    bool result = slq.compute(Q, R, A, B, K, P);
    if (!result) {
      K = control_feedback_t::Zero();
    }
    ////////////////////////////
    u = (-1) * K * next;
    ////////////////////////////
    if (u(0) > 40) {
      u(0) = 40;
    } else if (u(0) < -40) {
      u(0) = -40;
    }
    U.push_back(u);
    next = sys.runge_kutta_step(next, u);
    sys.linearize_dynamics(A, B, next, u);
  }

  P = Q * 2.0;
  control_t bb;
  for (int i = 0; i < 1000; ++i) {
    bb = U[0];
    state_t result_x = slq.solve_slq(Q, R, W, P, U, x0, goal_state);
    std::cout << "Before: " << bb << std::endl;
    std::cout << "After: " << U[0] << std::endl << std::endl;
    x0 = result_x;
    sys.linearize_dynamics(A, B, x0, U[0]);
    next = result_x;

    for (int j = 0; j < 200; ++j) {
      result = slq.compute(Q, R, A, B, K, P);
      if (!result) {
        K = control_feedback_t::Zero();
      }
      ///////////////////////////
      u = (-1) * K * next;
      ///////////////////////////
      if (u(0) > 40) {
        u(0) = 40;
      } else if (u(0) < -40) {
        u(0) = -40;
      }
      U[j] = u;
      next = sys.runge_kutta_step(next, u);
      sys.linearize_dynamics(A, B, next, u);
    }
  }
}

int main() {
  // run_slq_cartpole();
  run_lqr();
}
