/*
Robust solver class
author: Yun Chang, Luca Carlone
*/

#include "KimeraRPGO/RobustSolver.h"

#include <vector>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/logger.h"
#include "KimeraRPGO/outlier/pcm.h"

namespace KimeraRPGO {

RobustSolver::RobustSolver(const RobustSolverParams& params)
    : GenericSolver(params.solver, params.specialSymbols) {
  switch (params.outlierRemovalMethod) {
    case OutlierRemovalMethod::NONE: {
      outlier_removal_ =
          nullptr;  // only returns optimize true or optimize false
    } break;
    case OutlierRemovalMethod::PCM2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm2D>(params.pcm_odomThreshold,
                                         params.pcm_lcThreshold,
                                         params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm3D>(params.pcm_odomThreshold,
                                         params.pcm_lcThreshold,
                                         params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM_Simple2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple2D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM_Simple3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple3D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.specialSymbols);
    } break;
    default: {
      log<WARNING>("Undefined outlier removal method");
      exit(EXIT_FAILURE);
    }
  }

  // toggle verbosity
  switch (params.verbosity) {
    case Verbosity::UPDATE: {
      if (outlier_removal_) outlier_removal_->setQuiet();
    } break;
    case Verbosity::QUIET: {
      if (outlier_removal_)
        outlier_removal_->setQuiet();  // set outlier removal quiet
      setQuiet();                      // set solver quiet
    } break;
    case Verbosity::VERBOSE: {
      log<INFO>("Starting RobustSolver.");
    } break;
    default: {
      log<WARNING>("Unrecognized verbosity. Automatically setting to UPDATE. ");
    }
  }
}

void RobustSolver::optimize() {
  if (solver_type_ == Solver::LM) {
    gtsam::LevenbergMarquardtParams params;
    if (debug_) {
      params.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM");
    }
    params.diagonalDamping = true;
    values_ =
        gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
  } else if (solver_type_ == Solver::GN) {
    gtsam::GaussNewtonParams params;
    if (debug_) {
      params.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
  } else {
    log<WARNING>("Unsupported Solver");
    exit(EXIT_FAILURE);
  }
}

void RobustSolver::updateOnce(const gtsam::NonlinearFactorGraph& nfg,
                              const gtsam::Values& values) {
  // loop closures/outlier rejection
  bool process_lc;
  if (outlier_removal_) {
    process_lc = outlier_removal_->removeOutliers(nfg, values, nfg_, values_);
  } else {
    process_lc = addAndCheckIfOptimize(nfg, values);  // use default process
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

void RobustSolver::addOdometry(const gtsam::NonlinearFactorGraph& odom_factor,
                               const gtsam::Values& odom_values) {
  if (odom_factor.size() != 1 || odom_values.size() > 1) {
    log<WARNING>(
        "RobustSolver::addOdometry expects single factor and single value.");
  }
  if (outlier_removal_) {
    outlier_removal_->removeOutliers(odom_factor, odom_values, nfg_, values_);
  } else {
    addAndCheckIfOptimize(odom_factor, odom_values);
  }
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& factors,
                          const gtsam::Values& values) {
  // If no values and only loop closures, can just update
  if (values.size() == 0 || (values.size() == 1 && factors.size() == 1)) {
    updateOnce(factors, values);
    return;
  }

  // Sort ito different categories 
  std::unordered_map<char, gtsam::NonlinearFactorGraph> intra_robot_graphs; 
  gtsam::NonlinearFactorGraph landmark_factors; 
  gtsam::NonlinearFactorGraph inter_robot_factors; 

  for (size_t i = 0; i < factors.size(); i++) {
    if (factors[i] != NULL) {
      if (factors[i]->keys().size() == 1) {
        // prior factor 
      } else if (factor[i]->keys().size() == 2) {
        gtsam::Symbol symb_front(factors[i]->front());
        gtsam::Symbol symb_baack(factors[i]->back());
        if (symb_front.chr() != symb_back.chr()) {
          // check if the prefixes on the two keys are the same 
          if (isSpecialSymbol(symb_front.chr()) || isSpecialSymbol(symb_back.chr())) {
            // if one of them is a special symbol, must be a landmark factor 
            landmark_factors.add(factors[i]);
          } else {
            // if not a landmark factor and connects two diferent prefixes: intterrobot LC 
            inter_robot_factors.add(factors[i]);
          }
        } else {
          // check if prefix already exists 
          if (intra_robot_graphs.find(symb_front.chr()) == intra_robot_graphs.end()) {
            // not yet exists, create new 
            gtsam::NonlinearFactorGraph new_graph; 
            new_graph.add(factors[i]);
            intra_robot_graphs[symb_front.chr()] = new_graph;
          } else {
            // already exists add to graph 
            intra_robot_graphs[symb_front.chr()].add(factors[i]);
          }
        }
      }
    }
  }

  for (auto it = intra_robot_graphs.begin(); it != intra_robot_graphs.end(); ++it) {
    // TODO(Yun) what to do with values? 
    do_optimize = updateIntrarobot(); 
  }

  // now search for the special symbols (i.e. artifacts)
  std::vector<gtsam::Key> landmarks;
  for (size_t i = 0; i < landmark_factors.size(); i++) {
    if (update_factors[i] != NULL) {
      if (values.exists(update_factors[i]->back())) {
        // check that landmark have not previously been seen
        if (std::find(landmarks.begin(), landmarks.end(), symb) ==
            landmarks.end()) {
          gtsam::Values new_values;
          gtsam::NonlinearFactorGraph new_factors;
          new_values.insert(update_factors[i]->back(),
                            values.at(update_factors[i]->back()));
          new_factors.add(update_factors[i]);
          landmarks.push_back(symb);

          // This is essentially addOdometry
          if (outlier_removal_) {
            do_optimize = outlier_removal_->removeOutliers(
                new_factors, new_values, nfg_, values_);
          } else {
            do_optimize = addAndCheckIfOptimize(new_factors, new_values);
          }

          update_factors[i].reset();
        }
      }
    }
  }

  // add the other non-odom loop closures
  gtsam::NonlinearFactorGraph new_factors;
  for (size_t i = 0; i < factors.size(); i++) {
    if (update_factors[i] != NULL) {
      // if (debug_) {
      //   std::cout << "loop closure: " << factors[i]->front() << ">" <<
      //   factors[i]->back() << std::endl;
      // }
      new_factors.add(update_factors[i]);
    }
  }

  if (outlier_removal_) {
    do_optimize = outlier_removal_->removeOutliers(
        new_factors, gtsam::Values(), nfg_, values_);
  } else {
    do_optimize = addAndCheckIfOptimize(new_factors, gtsam::Values());
  }

  if (do_optimize) optimize();  // optimize once after loading
  return;
}  // namespace KimeraRPGO

void RobustSolver::saveData(std::string folder_path) const {
  std::string g2o_file_path = folder_path + "/result.g2o";
  gtsam::writeG2o(nfg_, values_, g2o_file_path);
  if (outlier_removal_) {
    outlier_removal_->saveData(folder_path);
  }
}

}  // namespace KimeraRPGO
