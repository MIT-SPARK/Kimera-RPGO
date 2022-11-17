/*
Generic backend solver class
author: Yun Chang, Luca Carlone
*/

#pragma once

#include <string>
#include <vector>

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/TypeUtils.h"

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

  inline gtsam::Values getTempValues() const { return temp_values_; }
  inline gtsam::NonlinearFactorGraph getTempFactorsUnsafe() const {
    return temp_nfg_;
  }
  inline void updateTempFactorsValues(
      const gtsam::NonlinearFactorGraph& temp_nfg,
      const gtsam::Values& temp_values) {
    temp_nfg_.add(temp_nfg);
    temp_values_.insert(temp_values);
  }
  inline void replaceTempFactorsValues(
      const gtsam::NonlinearFactorGraph& temp_nfg,
      const gtsam::Values& temp_values) {
    temp_nfg_ = temp_nfg;
    temp_values_ = temp_values;
  }
  inline void clearTempFactorsValues() {
    temp_nfg_ = gtsam::NonlinearFactorGraph();
    temp_values_ = gtsam::Values();
  }

  void print() const { values_.print(""); }

  void setQuiet() { debug_ = false; }

  EdgePtr removeLastFactor();  // remove last added factor

  void removePriorsWithPrefix(const char& prefix);

  void updateValues(const gtsam::Values& values);

 protected:
  bool addAndCheckIfOptimize(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values());

 protected:
  bool isSpecialSymbol(char symb) const;
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;
  // Factors and values subjected to change
  gtsam::Values temp_values_;
  gtsam::NonlinearFactorGraph temp_nfg_;

  Solver solver_type_;
  std::vector<char> special_symbols_;
  bool debug_;
  bool log_;
  std::string log_folder_;
};

}  // namespace KimeraRPGO