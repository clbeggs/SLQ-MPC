#ifndef QUADROTOR_H
#define QUADROTOR_H

#include <Eigen/Core>
#include <types.hpp>
#include <vector>

class Quadrotor {
 public:
  Quadrotor();
  ~Quadrotor();

  /*!
      @brief Forward dynamics of quadrotor

      From Paper: Full Control of a Quadrotor by Samir Bouabdallah and Roland
     Siegwart

     Equation (12)

  */
  state_t forward_dynamics(state_t &x_t, control_t &u_t);

  state_t forward_dynamics_inert(state_t &x_t, control_t &u_t,
                                 Eigen::Vector3f &I, Eigen::Vector4f &t);

  trajectory_t simulate_rollout(std::vector<control_t> &u,
                                state_t &initial_state);

  void integrate_trajectory(trajectory_t &traj);

  void cubic_spline_interpolation(
      Eigen::Matrix<float, Eigen::Dynamic, 4> &splines, float &FPO, float &FPN);

  void linearize_dynamics(state_matrix_t &A, control_gain_matrix_t &B,
                          state_t &x_t, control_t &u_t);

  void linearize_trajectory(trajectory_t &traj);

 protected:
  state_t _stable_state;
  control_t _hover;

 private:
  state_matrix_t calc_A(state_t &x, control_t &u);
  control_gain_matrix_t calc_B(state_t &x, control_t &u);

  const float pi = 3.14159265;

  const float mQ = 1.31;          // mass of quadcopter [ kg ]
  const float Thxxyy = 2.32e-3;   // moment of inertia around x,y [ kg*m^2 ]
  const float Thzz = 3e-4;        // moment of inertia around z [ kg*m^2 ]
  const float arm_len = 0.1;      // length of quadcopter arm [ m ]
  const float grav_const = 9.81;  // gravitational constant [ m/s^2 ]

  const float f_hover = mQ * grav_const;

  // Thrust parameters

  // kF = rotor thrust coefficient [ N/rad^2 ]
  const float kF = 6.17092e-8 * 3600 / (2 * pi * 2 * pi);

  // kM = rotor moment coefficient [ Nm/rad^2]
  const float kM = 1.3167e-9 * 3600 / (2 * pi * 2 * pi);

  const float wmax = 7800.0 * 2 * pi / 60;  // maximum rotor speed [ rad/s ]
  const float wmin = 1200.0 * 2 * pi / 60;  // minimum rotor speed [ rad/s ]
  const float Fsat_min = kF * wmin * wmin;
  const float Fsat_max = kF * wmax * wmax;

  // const Eigen::Vector4f kFs(kF, kF, kF, kF);
  // const Eigen::Vector4f kMs(kM, kM, kM, kM);

  // Thrust constants for props(found in proto file):
  // https://cyberbotics.com/doc/reference/propeller

  const float thrust_const_front_left_0 = 0.0000026;
  const float thrust_const_front_right_1 = -0.0000026;
  const float thrust_const_rear_left_2 = -0.0000026;
  const float thrust_const_rear_right_3 = 0.0000026;
};

#endif  // QUADROTOR_H
