/*
Generic solver class
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/RobustSolver.h"

namespace RobustPGO {

RobustSolver::RobustSolver(const RobustSolverParams& params) :
    GenericSolver(params.solver, params.specialSymbols) {
  switch (params.outlierRemovalMethod) {
    case OutlierRemovalMethod::NONE :
    {
      outlier_removal_ = nullptr; // only returns optimize true or optimize false
    }
    break;
    case OutlierRemovalMethod::PCM2D :
    {
      outlier_removal_ = std::make_unique<Pcm2D>(
          params.pcm_odomThreshold, params.pcm_lcThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM3D :
    {
      outlier_removal_ = std::make_unique<Pcm3D>(
          params.pcm_odomThreshold, params.pcm_lcThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM_Simple2D:
    {
      outlier_removal_ = std::make_unique<PcmSimple2D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM_Simple3D:
    {
      outlier_removal_ = std::make_unique<PcmSimple3D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold, params.specialSymbols);
    }
    break;
    default:
    {
      log<WARNING>("Undefined outlier removal method");
      exit (EXIT_FAILURE);
    }
  }

  // toggle verbosity
  switch (params.verbosity) {
    case Verbosity::UPDATE :
    {
      if (outlier_removal_) outlier_removal_->setQuiet();
    }
    break;
    case Verbosity::QUIET :
    {
      if (outlier_removal_) outlier_removal_->setQuiet(); // set outlier removal quiet
      setQuiet(); // set solver quiet
    }
    break;
    case Verbosity::VERBOSE :
    {
      log<INFO>("Starting RobustSolver.");
    }
    break;
    default:
    {
      log<WARNING>("Unrecognized verbosity. Automatically setting to UPDATE. ");
    }
  }
}

void RobustSolver::optimize() {
  if (solver_type_ == Solver::LM) {
    gtsam::LevenbergMarquardtParams params;
    if (debug_){
      params.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM");
    }
    params.diagonalDamping = true;
    values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
  }else if (solver_type_ == Solver::GN) {
    gtsam::GaussNewtonParams params;
    if (debug_) {
      params.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
  }else {
      log<WARNING>("Unsupported Solver");
      exit (EXIT_FAILURE);
  }
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values) {

  // loop closures/outlier rejection
  bool process_lc;
  if (outlier_removal_) {
    process_lc = outlier_removal_->removeOutliers(nfg, values, nfg_, values_);
  } else {
    process_lc = addAndCheckIfOptimize(nfg, values); // use default process
  }

  // optimize
  if (process_lc) {
    optimize();
  }
}

void RobustSolver::forceUpdate(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values) {

  if (outlier_removal_) {
    outlier_removal_->removeOutliers(nfg, values, nfg_, values_);
  } else {
    addAndCheckIfOptimize(nfg, values);
  }
  // optimize
  optimize();
}

void RobustSolver::addOdometry(const gtsam::NonlinearFactorGraph& odom_factor, const gtsam::Values& odom_values) {
  if (odom_factor.size() != 1 || odom_values.size() > 1) {
    log<WARNING>("RobustSolver::addOdometry expects single factor and single value.");
  }
  if (outlier_removal_) {
    outlier_removal_->removeOutliers(odom_factor, odom_values, nfg_, values_);
  } else {
    addAndCheckIfOptimize(odom_factor, odom_values);
  }
}

void RobustSolver::updateBatch(gtsam::NonlinearFactorGraph factors,
    const gtsam::Values& values, const gtsam::Key& key0) {

  // load graph assumes that the previous graph has been cleared
  gtsam::Key current_key = key0; // initial key
  // note that as of now only deal with between factors)
  // first load the odometry
  // order a nonlinear factor graph as odometry first
  bool extracted_odom = false;
  while (!extracted_odom) {
    bool end_of_odom = true;
    for (size_t i = 0; i < factors.size(); i++) {
      // search through
      if (factors[i] != NULL && factors[i]->keys().size() == 2 &&
          factors[i]->front() == current_key && factors[i]->back() == current_key + 1) {
        end_of_odom = false;

        gtsam::Values new_values;
        gtsam::NonlinearFactorGraph new_factors;
        // assumes key0 is already in the graph/values
        new_values.insert(current_key + 1, values.at(current_key + 1));
        new_factors.add(factors[i]);

        addOdometry(new_factors, new_values);

        current_key = current_key + 1;
        factors[i].reset(); // delete factor from graph
        break;
      }
    }
    if (end_of_odom) extracted_odom = true;
  }

  // now search for the special symbols (i.e. artifacts)
  std::vector<gtsam::Key> landmarks;
  for (size_t i = 0; i < factors.size(); i++) {
    if (factors[i] != NULL){
      gtsam::Symbol symb(factors[i]->back());
      if (isSpecialSymbol(symb.chr())) {
        // check that landmark have not previously been seen
        if (std::find(landmarks.begin(), landmarks.end(), symb) == landmarks.end()) {
          gtsam::Values new_values;
          gtsam::NonlinearFactorGraph new_factors;
          new_values.insert(factors[i]->back(), values.at(factors[i]->back()));
          new_factors.add(factors[i]);
          landmarks.push_back(symb);

          // This is essentially addOdometry, but let's not call it that here?
          // Since what's happening in outlier_removal_ is different
          if (outlier_removal_) {
            outlier_removal_->removeOutliers(new_factors, new_values, nfg_, values_);
          } else {
            addAndCheckIfOptimize(new_factors, new_values);
          }

          factors[i].reset();
        }
      }
    }
  }

  // add the other non-odom loop closures
  gtsam::NonlinearFactorGraph new_factors;
  for (size_t i = 0; i < factors.size(); i++) {
    if (factors[i] != NULL) {
      // if (debug_) {
      //   std::cout << "loop closure: " << factors[i]->front() << ">" << factors[i]->back() << std::endl;
      // }
      new_factors.add(factors[i]);
    }
  }

  if (outlier_removal_) {
    outlier_removal_->removeOutliers(new_factors, gtsam::Values(), nfg_, values_);
  } else {
    addAndCheckIfOptimize(new_factors, gtsam::Values());
  }

  optimize(); // optimize once after loading
}

}
