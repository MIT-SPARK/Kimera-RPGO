/*
Generic backend solver class
author: Yun Chang, Luca Carlone
*/

#ifndef KIMERARPGO_GENERICSOLVER_H_
#define KIMERARPGO_GENERICSOLVER_H_

#include <vector>

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/logger.h"

namespace KimeraRPGO {

class GenericSolver {
 public:
  GenericSolver(Solver solvertype = Solver::LM,
                std::vector<char> special_symbols = std::vector<char>());
  // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton
  // special symbols denote non odometry factors - perhaps semantics

  virtual ~GenericSolver() = default;

  void update(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values(),
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());

  void removeFactorsNoUpdate(
      gtsam::FactorIndices factorsToRemove = gtsam::FactorIndices());

  size_t size() { return nfg_.size(); }

  inline gtsam::Values calculateEstimate() const { return values_; }
  inline gtsam::Values calculateBestEstimate() const { return values_; }
  inline gtsam::Values getLinearizationPoint() const { return values_; }
  inline gtsam::NonlinearFactorGraph getFactorsUnsafe() const { return nfg_; }

  void print() const { values_.print(""); }

  void setQuiet() { debug_ = false; }

  void removeLastFactor();  // remove last added factor

 protected:
  bool addAndCheckIfOptimize(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values());

 protected:
  bool isSpecialSymbol(char symb) const;
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;
  Solver solver_type_;
  std::vector<char> special_symbols_;
  bool debug_;
};

}  // namespace KimeraRPGO

#endif  // KIMERARPGO_GENERICSOLVER_H_
