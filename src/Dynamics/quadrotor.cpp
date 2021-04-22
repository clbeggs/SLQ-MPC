#include <math.h>

#include <Eigen/Core>
#include <iostream>
#include <quadrotor.hpp>
#include <vector>

/////////////////////////////////
/////// Utility Functions ///////
/////////////////////////////////

Quadrotor::Quadrotor()
    : _stable_state((state_t() << 1.0, 1.0, 1.0, -1.5707943, 0.0636659,
                     -1.047188, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        .finished()),
      _hover((control_t() << 0, 0, 0, 0).finished()) {
}
Quadrotor::~Quadrotor() {
}

state_t Quadrotor::forward_dynamics(state_t &x_t, control_t &u_t) {
  // positions
  //    float qxQ = x(0);  // x
  //    float qyQ = x(1);  // y
  //    float qzQ = x(2);  // z

  // euler angles xyz
  float qph = x_t(3);
  float qth = x_t(4);
  float qps = x_t(5);

  // positions derivatives
  float dqxQ = x_t(6);  // x
  float dqyQ = x_t(7);  // y
  float dqzQ = x_t(8);  // z

  // euler angle derivatives xyz
  float dqph = x_t(9);
  float dqth = x_t(10);
  float dqps = x_t(11);

  // Calculate individual motor thrust
  // float thrust_0 = fmax(0, fmin(u_t(0), 10)) * thrust_const_front_left_0 *
  // abs(thrust_const_front_left_0);

  // float thrust_1 = fmax(0, fmin(u_t(1), 10)) * thrust_const_front_right_1 *
  // abs(thrust_const_front_right_1);

  // float thrust_2 = fmax(0, fmin(u_t(2), 10)) * thrust_const_rear_left_2 *
  // abs(thrust_const_rear_left_2);

  // float thrust_3 = fmax(0, fmin(u_t(3), 10)) * thrust_const_rear_right_3 *
  // abs(thrust_const_rear_right_3);

  // Calculate thrust
  // From section II A. of:
  // Unified Motion Control for Dynamic Quadrotor ManeuversDemonstrated on Slung
  // Load and Rotor Failure Tasks
  // fmax(0, fmin(u_t(3), 10))
  float max_u, min_u;
  max_u = 10;
  min_u = 0;
  float u1, u2, u3, u4;
  u1 = fmax(min_u, fmax(max_u, u_t(0)));
  u2 = fmax(min_u, fmax(max_u, u_t(1)));
  u3 = fmax(min_u, fmax(max_u, u_t(2)));
  u4 = fmax(min_u, fmax(max_u, u_t(3)));

  float thrust_0 = pow(u1, 2) * kM;
  float thrust_1 = pow(u2, 2) * kM;
  float thrust_2 = pow(u3, 2) * kM;
  float thrust_3 = pow(u4, 2) * kM;

  // Applied force and momentums
  float Fz = thrust_0 + thrust_1 + thrust_2 + thrust_3;  // total rotor thrust

  // Moments:
  // Source: Demystifying Drone Dynamics! by Percy Jaiswal
  // towards data science article

  float Mx = 0;
  float My = Mx;
  float Mz = Mx;

  float t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;

  t2 = 1.0 / mQ;
  t3 = std::cos(qth);
  t4 = std::sin(qth);
  t5 = 1.0 / Thxxyy;
  t6 = std::cos(qps);
  t7 = std::sin(qps);
  t8 = dqph * dqph;
  t9 = qth * 2.0;
  t10 = std::sin(t9);
  t11 = 1.0 / t3;
  t12 = Thzz * Thzz;
  t13 = t3 * t3;

  state_t dx;
  dx.setZero();

  dx(0) = dqxQ;
  dx(1) = dqyQ;
  dx(2) = dqzQ;
  dx(3) = dqph;
  dx(4) = dqth;
  dx(5) = dqps;
  dx(6) = Fz * t2 * t4;
  dx(7) = -Fz * t2 * t3 * std::sin(qph);
  dx(8) = t2 * (mQ * 9.81E2 - Fz * t3 * std::cos(qph) * 1.0E2) * (-1.0 / 1.0E2);
  dx(9) = -t5 * t11 *
          (-Mx * t6 + My * t7 + Thzz * dqps * dqth -
           Thxxyy * dqph * dqth * t4 * 2.0 + Thzz * dqph * dqth * t4);
  dx(10) = t5 * (Mx * t7 + My * t6 - Thxxyy * t8 * t10 * (1.0 / 2.0) +
                 Thzz * t8 * t10 * (1.0 / 2.0) + Thzz * dqph * dqps * t3);
  dx(11) = (t5 * t11 *
            (Mz * Thxxyy * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 +
             dqps * dqth * t4 * t12 - Thxxyy * Thzz * dqph * dqth * 2.0 -
             Mx * Thzz * t4 * t6 + My * Thzz * t4 * t7 +
             Thxxyy * Thzz * dqph * dqth * t13)) /
           Thzz;

  // a hacky check to prevent integration from becoming unstable:
  if (x_t.norm() > 1e20) dx.setZero();

  return dx;
}

state_matrix_t Quadrotor::calc_A(state_t &x, control_t &u) {
  float qph = x(3);
  float qth = x(4);
  float qps = x(5);
  float dqph = x(9);
  float dqth = x(10);
  float dqps = x(11);

  float Fz = u(0);
  float Mx = u(1);
  float My = u(2);

  float t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17,
      t18, t19, t20, t21, t22, t23, t24, t25;

  t2 = 1.0 / mQ;
  t3 = cos(qth);
  t4 = sin(qph);
  t5 = cos(qph);
  t6 = sin(qth);
  t7 = 1.0 / Thxxyy;
  t8 = cos(qps);
  t9 = sin(qps);
  t10 = 1.0 / t3;
  t11 = Thxxyy * 2.0;
  t12 = Thzz - t11;
  t13 = qth * 2.0;
  t14 = cos(t13);
  t15 = My * t9;
  t16 = sin(t13);
  t17 = 1.0 / (t3 * t3);
  t18 = qth * 3.0;
  t19 = sin(t18);
  t20 = My * t8;
  t21 = Mx * t9;
  t22 = t20 + t21;
  t23 = t3 * t3;
  t24 = t6 * t6;
  t25 = Thzz * dqps * t6;

  state_matrix_t A;
  A.setZero();

  A(0, 6) = 1.0;
  A(1, 7) = 1.0;
  A(2, 8) = 1.0;
  A(3, 9) = 1.0;
  A(4, 10) = 1.0;
  A(5, 11) = 1.0;
  A(6, 4) = Fz * t2 * t3;
  A(7, 3) = -Fz * t2 * t3 * t5;
  A(7, 4) = Fz * t2 * t4 * t6;
  A(8, 3) = -Fz * t2 * t3 * t4;
  A(8, 4) = -Fz * t2 * t5 * t6;
  A(9, 4) = -t6 * t7 * t17 *
                (t15 - Mx * t8 + Thzz * dqps * dqth -
                 Thxxyy * dqph * dqth * t6 * 2.0 + Thzz * dqph * dqth * t6) -
            dqph * dqth * t7 * t12;
  A(9, 5) = -t7 * t10 * t22;
  A(9, 9) = -dqth * t6 * t7 * t10 * t12;
  A(9, 10) =
      -t7 * t10 * (Thzz * dqps - Thxxyy * dqph * t6 * 2.0 + Thzz * dqph * t6);
  A(9, 11) = -Thzz * dqth * t7 * t10;
  A(10, 4) = -dqph * t7 * (t25 + Thxxyy * dqph * t14 - Thzz * dqph * t14);
  A(10, 5) = -t7 * (t15 - Mx * t8);
  A(10, 9) = t7 * (-Thxxyy * dqph * t16 + Thzz * dqps * t3 + Thzz * dqph * t16);
  A(10, 11) = Thzz * dqph * t3 * t7;
  A(11, 4) = t7 * t17 *
             (Mx * t8 * -4.0 + My * t9 * 4.0 + Thzz * dqps * dqth * 4.0 -
              Thxxyy * dqph * dqth * t6 * 9.0 - Thxxyy * dqph * dqth * t19 +
              Thzz * dqph * dqth * t6 * 5.0 + Thzz * dqph * dqth * t19) *
             (1.0 / 4.0);
  A(11, 5) = t6 * t7 * t10 * t22;
  A(11, 9) = dqth * t7 * t10 * (Thzz - t11 + Thxxyy * t23 - Thzz * t23);
  A(11, 10) = t7 * t10 *
              (t25 - Thxxyy * dqph - Thxxyy * dqph * t24 + Thzz * dqph * t24);
  A(11, 11) = Thzz * dqth * t6 * t7 * t10;

  return A;
}
control_gain_matrix_t Quadrotor::calc_B(state_t &x, control_t &u) {
  float qph = x(3);
  float qth = x(4);
  float qps = x(5);

  float t2, t3, t4, t5, t6, t7, t8;

  t2 = 1.0 / mQ;
  t3 = cos(qth);
  t4 = 1.0 / Thxxyy;
  t5 = 1.0 / t3;
  t6 = sin(qps);
  t7 = cos(qps);
  t8 = sin(qth);

  control_gain_matrix_t B;
  B.setZero();

  B(6, 0) = t2 * t8;
  B(7, 0) = -t2 * t3 * sin(qph);
  B(8, 0) = t2 * t3 * cos(qph);
  B(9, 1) = t4 * t5 * t7;
  B(9, 2) = -t4 * t5 * t6;
  B(10, 1) = t4 * t6;
  B(10, 2) = t4 * t7;
  B(11, 1) = -t4 * t5 * t7 * t8;
  B(11, 2) = t4 * t5 * t6 * t8;
  B(11, 3) = 1.0 / Thzz;

  return B;
}

void Quadrotor::linearize_dynamics(state_matrix_t &A, control_gain_matrix_t &B,
                                   state_t &x_t, control_t &u_t) {
  A = calc_A(x_t, u_t);
  B = calc_B(x_t, u_t);
}

void Quadrotor::linearize_trajectory(trajectory_t &traj) {
  for (int i = 0; i < traj.x.size(); i++) {
    this->linearize_dynamics(traj.A[i], traj.B[i], traj.x[i], traj.u[i]);
  }
}

void Quadrotor::cubic_spline_interpolation(
    Eigen::Matrix<float, Eigen::Dynamic, 4> &splines, float &FPO, float &FPN) {
  // vars.row(0) = alpha
  // vars.row(1) = l_i
  // vars.row(2) = z_i
  // vars.row(3) = mu_i

  // float h = 0.001;
  // int N = splines.rows();
  // Eigen::Matrix<float, Eigen::Dynamic, 4> vars(N, 4);

  // for (int i = 0; i < N; ++i) {
  // }
}

void Quadrotor::integrate_trajectory(trajectory_t &traj) {
  // Cubic splines interpolation
  int N = traj.x.size();
  // Eigen::Matrix<float, Eigen::Dynamic, 4> splines(N);
  // cubic_spline_interpolation(splines);

  // runge kutta
}

trajectory_t Quadrotor::simulate_rollout(std::vector<control_t> &u,
                                         state_t &x0) {
  int N = u.size();
  trajectory_t traj(N, x0);

  for (int i = 0; i < u.size() - 1; i++) {
    traj.x[i + 1] = forward_dynamics(traj.x[i], u[i]);
    traj.u[i] = u[i];
  }

  return traj;
}

state_t Quadrotor::forward_dynamics_inert(state_t &x_t, control_t &u_t,
                                          Eigen::Vector3f &inertia_diag,
                                          Eigen::Vector4f &thrust_const) {
  // u_t is angular speeds of rotos in rad / sec

  // positions
  float x = x_t(0);  // x
  float y = x_t(1);  // y
  float z = x_t(2);  // z

  // euler angles xyz
  float phi = x_t(3);
  float theta = x_t(4);
  float psi = x_t(5);

  // positions derivatives
  float dx = x_t(6);  // x
  float dy = x_t(7);  // y
  float dz = x_t(8);  // z

  // euler angle derivatives xyz
  float dphi = x_t(9);
  float dtheta = x_t(10);
  float dpsi = x_t(11);

  state_t next_state = state_t::Zero();
  // linear velocity
  next_state(0) = dx;
  next_state(1) = dy;
  next_state(2) = dz;
  // angular velocity
  next_state(3) = dphi;
  next_state(4) = dtheta;
  next_state(5) = dpsi;

  // Angular speeds of rotors
  // float w1 = u_t(0);
  // float w2 = u_t(1);
  // float w3 = u_t(2);
  // float w4 = u_t(3);

  // Compute vertical forces from rotors (3)
  // float F_1 = kF * pow(w1, 2);
  // float F_2 = kF * pow(w2, 2);
  // float F_3 = kF * pow(w3, 2);
  // float F_4 = kF * pow(w4, 2);

  // // Compute moments from rotors (4)
  // float M_1 = kM * pow(w1, 2);
  // float M_2 = kM * pow(w2, 2);
  // float M_3 = kM * pow(w3, 2);
  // float M_4 = kM * pow(w4, 2);

  float total_thrust = (thrust_const(0) * abs(u_t(0)) * u_t(0)) +
                       (thrust_const(1) * abs(u_t(1)) * u_t(1)) +
                       (thrust_const(2) * abs(u_t(2)) * u_t(2)) +
                       (thrust_const(3) * abs(u_t(3)) * u_t(3));

  // Computing linear acceleration
  float fm = -(total_thrust / mQ);
  printf("CALCULATION===========================\n");

  std::cout << u_t << std::endl;
  std::cout << fm << std::endl;
  std::cout << thrust_const(0) * abs(u_t(0)) << std::endl;
  std::cout << thrust_const(1) * abs(u_t(1)) << std::endl;
  std::cout << thrust_const(2) * abs(u_t(2)) << std::endl;
  std::cout << thrust_const(3) * abs(u_t(3)) << std::endl;

  std::cout << (sin(phi) * sin(psi) + cos(phi) * cos(phi) * sin(theta))
            << std::endl;

  next_state(6) = fm * (sin(phi) * sin(psi) + cos(phi) * cos(phi) * sin(theta));
  next_state(7) = fm * (cos(phi) * sin(psi) - cos(psi) * sin(phi));
  next_state(8) = grav_const - fm * (cos(phi) * cos(theta));

  // next_state(6) = (dpsi * dy) - (dtheta * dz) - this->grav_const *
  // sin(theta); next_state(7) = (dtheta * dz) - (dpsi * dx) +
  // this->grav_const
  // * (sin(phi) * cos(theta));

  // next_state(8) =
  // (dtheta * dx) - (dphi * dy) + this->grav_const * (cos(theta) *
  // cos(phi));

  float drag = 1.0;  // TODO: change

  // // computing angular acceleration
  float tau_x =
      thrust_const(0) * this->arm_len * (pow(u_t(2), 2) - pow(u_t(0), 2));
  float tau_y =
      thrust_const(0) * this->arm_len * (pow(u_t(3), 2) - pow(u_t(1), 2));
  float tau_z = drag * (pow(u_t(1), 2) + pow(u_t(3), 2) - pow(u_t(0), 2) -
                        pow(u_t(2), 2));

  next_state(9) = ((inertia_diag(1) - inertia_diag(2)) / inertia_diag(0)) *
                      (dtheta * dpsi) +
                  (tau_x / inertia_diag(0));
  next_state(10) =
      ((inertia_diag(2) - inertia_diag(0)) / inertia_diag(1)) * (dphi * dpsi) +
      (tau_y / inertia_diag(1));

  next_state(11) = ((inertia_diag(0) - inertia_diag(1)) / inertia_diag(2)) *
                       (dtheta * dphi) +
                   (tau_z / inertia_diag(2));

  return next_state;
}
