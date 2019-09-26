/*
Generic backend solver class
author: Yun Chang, Luca Carlone
*/

#ifndef GENERICSOLVER_H
#define GENERICSOLVER_H

#include <vector>

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "RobustPGO/logger.h"
#include "RobustPGO/SolverParams.h"

namespace RobustPGO {

class GenericSolver {
public:
  GenericSolver(Solver solvertype=Solver::LM,
                std::vector<char> special_symbols=std::vector<char>());
  // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton
  // special symbols denote non odometry factors - perhaps semantics

  virtual ~GenericSolver() = default;

  void update(const gtsam::NonlinearFactorGraph& nfg=gtsam::NonlinearFactorGraph(),
              const gtsam::Values& values=gtsam::Values(),
              const gtsam::FactorIndices& factorsToRemove=gtsam::FactorIndices());

  void removeFactorsNoUpdate(
      gtsam::FactorIndices factorsToRemove = gtsam::FactorIndices());

  gtsam::Values calculateEstimate() { return values_; }
  gtsam::Values calculateBestEstimate() { return values_; }
  gtsam::Values getLinearizationPoint() { return values_; }
  gtsam::NonlinearFactorGraph getFactorsUnsafe(){ return nfg_; }

  void print() const {
    values_.print("");
  }

  void setQuiet() { debug_ = false; }

protected:
  bool addAndCheckIfOptimize(const gtsam::NonlinearFactorGraph& nfg=gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values=gtsam::Values());

protected:
  bool isSpecialSymbol(char symb) const;
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;
  Solver solver_type_;
  std::vector<char> special_symbols_;
  bool debug_;
};

}

#endif