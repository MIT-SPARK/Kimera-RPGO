/* 
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
*/

#ifndef ROBUSTPGO_H
#define ROBUSTPGO_H

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
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "RobustPGO/GenericSolver.h"
#include "RobustPGO/logger.h"
#include "RobustPGO/OutlierRemoval.h"

class RobustPGO : public GenericSolver{
public:
  RobustPGO(OutlierRemoval* OR,
            int solvertype=1, 
            std::vector<char> special_symbols=std::vector<char>());
      // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton

  void update(gtsam::NonlinearFactorGraph nfg=gtsam::NonlinearFactorGraph(), 
              gtsam::Values values=gtsam::Values(),
              gtsam::FactorIndices factorsToRemove=gtsam::FactorIndices());

  void force_optimize(); 

private:
  OutlierRemoval* outlier_removal_; // outlier removal method; 

  void optimize(); 
};

#endif