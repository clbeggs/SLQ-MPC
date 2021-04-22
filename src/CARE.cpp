#include <CARE.hpp>
#include <Eigen/Core>
#include <types.hpp>

CARE::CARE() {
}
CARE::~CARE() {
}

bool CARE::solve_iterative(schur_matrix_t &M, state_matrix_t &P, int max_iters,
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
  /* break down W and extract W11 W12 W21 W22  (what is the size of these?) */
  state_matrix_t M11(Mlocal.template block<12, 12>(0, 0));
  state_matrix_t M12(Mlocal.template block<12, 12>(0, 12));
  state_matrix_t M21(Mlocal.template block<12, 12>(12, 0));
  state_matrix_t M22(Mlocal.template block<12, 12>(12, 12));

  /* find M and N using the elements of W	 */
  factor_matrix_t U;
  factor_matrix_t V;

  U.template block<12, 12>(0, 0) = M12;
  U.template block<12, 12>(12, 0) = M22 + state_matrix_t::Identity();

  V.template block<12, 12>(0, 0) = M11 + state_matrix_t::Identity();
  V.template block<12, 12>(12, 0) = M21;

  /* Solve for S from the equation   MS=N */
  FullPivLU_.compute(U);

  P = FullPivLU_.solve(-V);

  return true;
}

bool CARE::solve_direct(schur_matrix_t &M, state_matrix_t &P) {
  return false;
}

bool CARE::solve_care(state_matrix_t &Q, control_matrix_t &R, state_matrix_t &A,
                      control_gain_matrix_t &B, state_matrix_t &P,
                      control_matrix_t &Rinv, bool use_iterative) {
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
    return false;
}
