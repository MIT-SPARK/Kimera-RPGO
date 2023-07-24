/*
Generic solver class
No outlier removal in this class
author: Yun Chang, Luca Carlone
*/

#include "KimeraRPGO/GenericSolver.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

namespace KimeraRPGO {

GenericSolver::GenericSolver(Solver solvertype,
                             std::vector<char> special_symbols)
    : values_(gtsam::Values()),
      nfg_(gtsam::NonlinearFactorGraph()),
      solver_type_(solvertype),
      special_symbols_(special_symbols),
      debug_(true),
      log_(false) {}

bool GenericSolver::isSpecialSymbol(char symb) const {
  for (size_t i = 0; i < special_symbols_.size(); i++) {
    if (special_symbols_[i] == symb) return true;
  }
  return false;
}

void GenericSolver::updateValues(const gtsam::Values& values) {
  for (const auto& v : values) {
    if (values_.exists(v.key)) {
      values_.update(v.key, v.value);
    } else if (temp_values_.exists(v.key)) {
      temp_values_.update(v.key, v.value);
    }
  }
}

bool GenericSolver::addAndCheckIfOptimize(
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  // add new values and factors
  nfg_.add(nfg);
  values_.insert(values);

  // Do not optimize for just odometry (between) additions
  if (nfg.size() == 1 && nfg[0]->keys().size() == 2 && values.size() == 1) {
    return false;
  }

  // nothing added so no optimization
  if (nfg.size() == 0 && values.size() == 0) {
    return false;
  }

  return true;
}

void GenericSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values,
                           const gtsam::FactorIndices& factorsToRemove) {
  // TODO(Yun) Do we have unittests for generic (no outlier-rejection) update?
  // remove factors
  bool remove_factors = false;
  if (factorsToRemove.size() > 0) {
    remove_factors = true;
  }
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  bool process_lc = addAndCheckIfOptimize(nfg, values);

  if (process_lc || remove_factors) {
    // optimize
    gtsam::Values result;
    gtsam::Values full_values = values_;
    gtsam::NonlinearFactorGraph full_nfg = nfg_;
    full_values.insert(temp_values_);
    full_nfg.add(temp_nfg_);
    if (solver_type_ == Solver::LM) {
      gtsam::LevenbergMarquardtParams params;
      if (debug_) {
        params.setVerbosityLM("SUMMARY");
        log<INFO>("Running LM");
      }
      params.diagonalDamping = true;
      result = gtsam::LevenbergMarquardtOptimizer(full_nfg, full_values, params)
                   .optimize();
    } else if (solver_type_ == Solver::GN) {
      gtsam::GaussNewtonParams params;
      if (debug_) {
        params.setVerbosity("ERROR");
        log<INFO>("Running GN");
      }
      result =
          gtsam::GaussNewtonOptimizer(full_nfg, full_values, params).optimize();
    } else {
      log<WARNING>("Unsupported Solver");
      exit(EXIT_FAILURE);
    }
    updateValues(result);
  }
}

void GenericSolver::removeFactorsNoUpdate(
    gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }
}

EdgePtr GenericSolver::removeLastFactor() {
  size_t num_factors = nfg_.size();
  Edge removed_edge =
      Edge(nfg_[num_factors - 1]->front(), nfg_[num_factors - 1]->back());
  nfg_.erase(std::prev(nfg_.end()));
  return make_unique<Edge>(removed_edge);
}

void GenericSolver::removePriorsWithPrefix(const char& prefix) {
  // First make copy of nfg_
  const gtsam::NonlinearFactorGraph nfg_copy = nfg_;
  // Clear nfg_
  nfg_ = gtsam::NonlinearFactorGraph();
  // Iterate and pick out non prior factors and prior factors without key with
  // prefix
  for (auto factor : nfg_copy) {
    auto prior_factor_3d =
        factor_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior_factor_3d) {
      gtsam::Symbol node(prior_factor_3d->key());
      if (node.chr() != prefix) {
        nfg_.add(factor);
      }
      continue;
    }

    auto prior_factor_2d =
        factor_pointer_cast<gtsam::PriorFactor<gtsam::Pose2>>(factor);
    if (prior_factor_2d) {
      gtsam::Symbol node(prior_factor_2d->key());
      if (node.chr() != prefix) {
        nfg_.add(factor);
      }
      continue;
    }

    nfg_.add(factor);
  }
}

}  // namespace KimeraRPGO
