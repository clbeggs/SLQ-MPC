#include <mkl.h>

#include <CARE.hpp>
#include <Eigen/Core>
#include <iostream>
#include <types.hpp>

CARE::CARE() {
  schur_matrix_t T;
  schur_matrix_t U;
  int STATE_DIM = 4;

  int SELECT[2 * STATE_DIM];
  int N = 2 * STATE_DIM;
  double WR[T.ColsAtCompileTime];
  double WI[T.ColsAtCompileTime];
  int MS;
  double S;
  double SEP;
  double WORKDUMMY[1];
  int LWORK = -1;
  int IWORKQUERY[1];
  int LIWORK = -1;
  int INFO = 0;
  int TCols = schur_matrix_t::ColsAtCompileTime;

  dtrsen_("N", "V", &SELECT[0], &TCols, T.data(), &N, U.data(), &N, &WR[0],
          &WI[0], &MS, &S, &SEP, WORKDUMMY, &LWORK, &IWORKQUERY[0], &LIWORK,
          &INFO);

  LWORK_ = WORKDUMMY[0] + 32;
  LIWORK_ = IWORKQUERY[0] + 32;

  WORK_.resize(LWORK_);
  IWORK_.resize(LIWORK_);

  if (INFO != 0) {
    std::cout << "Lapack invocation of dtrsen failed!" << std::endl;
    exit(-1);
  }
}
CARE::~CARE() {
}

bool CARE::solve_iterative(schur_matrix_t& M, state_matrix_t& P, int max_iters,
                           float epsilon) {
  bool converged = false;

  schur_matrix_t Mlocal = M;
  int iters = 0;

  while (!converged) {
    if (iters > max_iters) return false;

    schur_matrix_t Mdiff = Mlocal - Mlocal.inverse();
    schur_matrix_t Mnew = Mlocal - 0.5 * Mdiff;

    converged = Mnew.isApprox(Mlocal, epsilon);

    Mlocal = Mnew;

    iters++;
  }
  /* break down W and extract W11 W4 W21 W22  (what is the size of these?) */
  state_matrix_t M11(Mlocal.template block<4, 4>(0, 0));
  state_matrix_t M4(Mlocal.template block<4, 4>(0, 4));
  state_matrix_t M21(Mlocal.template block<4, 4>(4, 0));
  state_matrix_t M22(Mlocal.template block<4, 4>(4, 4));

  /* find M and N using the elements of W	 */
  factor_matrix_t U;
  factor_matrix_t V;

  U.template block<4, 4>(0, 0) = M4;
  U.template block<4, 4>(4, 0) = M22 + state_matrix_t::Identity();

  V.template block<4, 4>(0, 0) = M11 + state_matrix_t::Identity();
  V.template block<4, 4>(4, 0) = M21;

  /* Solve for S from the equation   MS=N */
  FullPivLU_.compute(U);

  P = FullPivLU_.solve(-V);

  return true;
}

bool CARE::solve_direct(schur_matrix_t& M, state_matrix_t& P) {
  const bool computeU = true;
  schur_.compute(M, computeU);

  if (schur_.info() != Eigen::Success) {
    throw std::runtime_error(
        "LQR Schur computation failed. Most likely problem is set up wrongly "
        "or not solvable.");
  }

  schur_matrix_t U(schur_.matrixU());
  schur_matrix_t T(schur_.matrixT());

  int STATE_DIM = 4;
  int SELECT[2 * STATE_DIM];
  double WR[2 * STATE_DIM];
  double WI[2 * STATE_DIM];
  int MS;
  double S;
  double SEP;
  int INFO = 0;
  int N = 2 * STATE_DIM;

  for (size_t i = 0; i < 2 * STATE_DIM; i++) {
    // check if last row or eigenvalue is complex (2x2 block)
    if (i == (2 * STATE_DIM - 1) || std::abs(T(i + 1, i)) < 1e-12) {
      SELECT[i] = static_cast<int>(T(i, i) < 0);
    } else {
      // we have a complex block
      SELECT[i] = static_cast<int>((T(i, i) + T(i + 1, i + 1)) / 2.0 < 0);
      SELECT[i + 1] = SELECT[i];
      i++;
    }
  }

  dtrsen_("N", "V", &SELECT[0], &N, T.data(), &N, U.data(), &N, &WR[0], &WI[0],
          &MS, &S, &SEP, WORK_.data(), &LWORK_, IWORK_.data(), &LIWORK_, &INFO);

  const state_matrix_t& U11 = U.template block<4, 4>(0, 0);
  const state_matrix_t& U21 = U.template block<4, 4>(STATE_DIM, 0);

  // solve here for better numerical properties
  P.noalias() = U21 * U11.inverse();

  if (INFO != 0) {
    return false;
  }

  return true;
}

bool CARE::solve_care(state_matrix_t& Q, control_matrix_t& R, state_matrix_t& A,
                      control_gain_matrix_t& B, state_matrix_t& P,
                      control_matrix_t& Rinv, bool use_iterative) {
  // TODO: Check if R is diagonal
  // if (check_diagonal(R)) {
  // Rinv.setZero();
  // Rinv.diagonal().noalias() = R.diagonal().cwiseInverse();

  Rinv.noalias() = R.inverse();

  schur_matrix_t M;
  M << A, -B * Rinv * B.transpose(), -Q, -A.transpose();

  if (use_iterative)
    return solve_iterative(M, P, 1000, 1e-6);
  else
    return solve_direct(M, P);
}
