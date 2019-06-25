/* 
Generic backend solver class 
author: Yun Chang, Luca Carlone
*/

#ifndef GENERICSOLVER_H
#define GENERICSOLVER_H

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR 

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include "RobustPGO/logger.h"

class GenericSolver {
public:
  GenericSolver(int solvertype=1, 
                std::vector<char> special_symbols=std::vector<char>()); 
  // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton
  // special symbols denote non odometry factors - perhaps semantics 

  void update(gtsam::NonlinearFactorGraph nfg=gtsam::NonlinearFactorGraph(), 
              gtsam::Values values=gtsam::Values(),
              gtsam::FactorIndices factorsToRemove=gtsam::FactorIndices());

  void removeFactorsNoUpdate(
      gtsam::FactorIndices factorsToRemove = gtsam::FactorIndices());

  gtsam::Values calculateEstimate() { return values_; }
  gtsam::Values calculateBestEstimate() { return values_; }
  gtsam::Values getLinearizationPoint() { return values_; }
  gtsam::NonlinearFactorGraph getFactorsUnsafe(){ return nfg_; }

  void print() {
    values_.print("");
  }

protected:
  bool specialSymbol(char symb);
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;
  int solver_type_;
  std::vector<char> special_symbols_; 
};

#endif