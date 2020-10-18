/*
Example file to perform robust optimization on g2o files
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
#include "KimeraRPGO/logger.h"
#include "KimeraRPGO/utils/geometry_utils.h"
#include "KimeraRPGO/utils/type_utils.h"

using namespace KimeraRPGO;

/* Usage: ./RpgoReadG2o 2d <some-2d-g2o-file> <odom-threshold> <pcm-threshold>
   <output-g2o-file> <verbosity> [or]   ./RpgoReadG2o 3d <some-3d-g2o-file>
   <odom-threshold> <pcm-threshold> <output-g2o-file> <verbosity>*/
template <class T>
void Simulate(gtsam::GraphAndValues gv,
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
  pgo->update(nfg, values);

  pgo->saveData(output_folder);  // tell pgo to save g2o result
}

int main(int argc, char* argv[]) {
  gtsam::GraphAndValues graphNValues;
  std::string dim = argv[1];
  std::string output_folder;
  if (argc > 5) output_folder = argv[5];

  bool verbose = false;
  if (argc > 6) {
    std::string flag = argv[6];
    if (flag == "v") verbose = true;
  }
  RobustSolverParams params;

  params.logOutput(output_folder);

  Verbosity verbosity = Verbosity::VERBOSE;
  if (!verbose) verbosity = Verbosity::QUIET;

  if (dim == "2d") {
    graphNValues = gtsam::load2D(argv[2],
                                 gtsam::SharedNoiseModel(),
                                 0,
                                 false,
                                 true,
                                 gtsam::NoiseFormatG2O);

    params.setPcmSimple2DParams(atof(argv[3]), atof(argv[4]), verbosity);

    Simulate<gtsam::Pose2>(graphNValues, params, output_folder);

  } else if (dim == "3d") {
    graphNValues = gtsam::load3D(argv[2]);

    params.setPcmSimple3DParams(atof(argv[3]), atof(argv[4]), verbosity);

    Simulate<gtsam::Pose3>(graphNValues, params, output_folder);

  } else {
    log<WARNING>("Unsupported input format: ");
    log<WARNING>(
        "Should be ./RpgoReadG2o <2d or 3d> <g2o file> <odom thresh> "
        "<pcm thresh> <opt: output_folder> <opt: v for messages");
  }
}
