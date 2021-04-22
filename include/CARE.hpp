#ifndef CARE_H
#define CARE_H

#include <Dynamics.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <types.hpp>

class CARE {
 private:
  typedef Eigen::Matrix<float, 24, 24> schur_matrix_t;
  typedef Eigen::Matrix<float, 24, 12> factor_matrix_t;
  Eigen::RealSchur<schur_matrix_t> schur_;
  Eigen::FullPivLU<factor_matrix_t> FullPivLU_;
  int LWORK_;
  int LIWORK_;

  Eigen::VectorXd WORK_;
  Eigen::VectorXi IWORK_;

  bool solve_iterative(schur_matrix_t &M, state_matrix_t &P, int max_iters,
                       float epsilon);

  bool solve_direct(schur_matrix_t &M, state_matrix_t &P);

 public:
  CARE();
  ~CARE();
  /*!
     Q - intermediate cost
     R - input cost
     K - kalman gain

  */
  bool solve_care(state_matrix_t &Q, control_matrix_t &R, state_matrix_t &A,
                  control_gain_matrix_t &B, state_matrix_t &P,
                  control_matrix_t &Rinv, bool use_iterative);
};
#endif  // CARE_H
