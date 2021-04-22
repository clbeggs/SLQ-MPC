#ifndef MPC_H
#define MPC_H

#include <Eigen/Core>
#include <LQR.hpp>
#include <SLQ.hpp>
#include <Sim.hpp>
#include <types.hpp>
#include <vector>

class MPC : public SLQ {
 public:
  MPC();
  MPC(Sim_Interface *sim, state_t &x0);
  ~MPC();

  void run(state_t &goal_state);
  void run2(state_t &goal_state);
};

#endif  // MPC_H
