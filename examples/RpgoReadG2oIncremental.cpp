/*
Example file to perform robust optimization on g2o files but incrementally
author: Yun Chang
*/

#include <stdlib.h>
#include <memory>
#include <string>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/utils/GeometryUtils.h"
#include "KimeraRPGO/utils/TypeUtils.h"

using namespace KimeraRPGO;

/* Usage: ./RpgoReadG2o 2d <some-2d-g2o-file> <incremental> <odom-threshold>
   <pcm-threshold> <output-g2o-file> <verbosity> [or]*/
template <class T>
void SimulateIncremental(gtsam::GraphAndValues gv,
                         RobustSolverParams params,
                         std::string output_folder) {
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  size_t dim = getDim<T>();

  Eigen::VectorXd noise = Eigen::VectorXd::Zero(dim);
  static const gtsam::SharedNoiseModel& init_noise =
      gtsam::noiseModel::Diagonal::Sigmas(noise);

  gtsam::Key current_key = nfg[0]->front();

  gtsam::Values init_values;  // add first value with prior factor
  gtsam::NonlinearFactorGraph init_factors;
  init_values.insert(current_key, values.at<T>(current_key));
  gtsam::PriorFactor<T> prior_factor(
      current_key, values.at<T>(current_key), init_noise);
  nfg.add(prior_factor);

  // separate to non loop closures and loop closure factors
  gtsam::NonlinearFactorGraph non_lc_factors, lc_factors;
  for (auto factor : nfg) {
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<T>>(factor)) {
      // specifically what outlier rejection handles
      gtsam::Key from_key = factor->front();
      gtsam::Key to_key = factor->back();
      if (from_key + 1 == to_key) {
        non_lc_factors.add(factor);  // odometry
      } else {
        lc_factors.add(factor);  // loop closure
      }
    } else {
      non_lc_factors.add(factor);  // not between so not lc
    }
  }
  // add non lc factors first
  pgo->update(non_lc_factors, values);

  // Now add loop closure one by one
  for (auto loop_closure : lc_factors) {
    gtsam::NonlinearFactorGraph new_factors;
    new_factors.add(loop_closure);
    pgo->update(new_factors, gtsam::Values(), false);
  }
  pgo->saveData(output_folder);  // tell pgo to save g2o result
}

int main(int argc, char* argv[]) {
  gtsam::GraphAndValues graphNValues;
  std::string dim = argv[1];
  int incremental = std::atoi(argv[3]);
  int gnc = std::atoi(argv[4]);
  int frame_align = std::atoi(argv[5]);
  std::string output_folder;
  if (argc > 8) output_folder = argv[8];

  bool verbose = false;
  if (argc > 9) {
    std::string flag = argv[9];
    if (flag == "v") verbose = true;
  }
  RobustSolverParams params;

  params.logOutput(output_folder);

  if (incremental == 1) {
    params.setIncremental();
  }

  if (gnc == 1) {
    params.setGncInlierCostThresholds(1.0);
  }

  if (frame_align == 1) {
    params.setMultiRobotAlignMethod(MultiRobotAlignMethod::GNC);
  }

  Verbosity verbosity = Verbosity::VERBOSE;
  if (!verbose) verbosity = Verbosity::QUIET;

  if (dim == "2d") {
    graphNValues = gtsam::load2D(argv[2],
                                 gtsam::SharedNoiseModel(),
                                 0,
                                 false,
                                 true,
                                 gtsam::NoiseFormatG2O);

    params.setPcmSimple2DParams(atof(argv[6]), atof(argv[7]), verbosity);

    SimulateIncremental<gtsam::Pose2>(graphNValues, params, output_folder);

  } else if (dim == "3d") {
    graphNValues = gtsam::load3D(argv[2]);

    params.setPcmSimple3DParams(atof(argv[6]), atof(argv[7]), verbosity);

    SimulateIncremental<gtsam::Pose3>(graphNValues, params, output_folder);

  } else {
    log<WARNING>("Unsupported input format: ");
    log<WARNING>(
        "Should be ./RpgoReadG2oIncremental <2d or 3d> <g2o file> <0 or 1 "
        "(incremental)> <0 or 1 (gnc)> <0 or 1 (multirobot frame alignment)> "
        "<trans thresh> rot thresh> <opt: "
        "output_folder> <opt: v for messages");
  }
}
