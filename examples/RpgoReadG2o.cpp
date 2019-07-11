/* 
Generic solver class 
author: Yun Chang
*/

#include "RobustPGO/RobustPGO.h"
#include "RobustPGO/pcm/pcm.h"
#include "RobustPGO/logger.h"

#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

#include <stdlib.h>

/* Usage: ./RpgoReadG2o 2d <some-2d-g2o-file> <odom-threshold> <pcm-threshold> <output-g2o-file> <verbosity>
   [or]   ./RpgoReadG2o 3d <some-3d-g2o-file> <odom-threshold> <pcm-threshold> <output-g2o-file> <verbosity>*/
template<class T>
void Simulate(gtsam::GraphAndValues gv, 
      double odom_thresh, double pmc_thresh, 
      std::string output_folder, 
      bool debug=true) {

  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;
  
  OutlierRemoval *pcm = new PCM<T>(odom_thresh, pmc_thresh);
  if (!debug) pcm->setQuiet();

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->saveG2oResult(output_folder); // tell pgo to save g2o result

  if (!debug) pgo->setQuiet(); // turn off print messages
  if (debug) log<INFO>("Initiated robust pose graph optimizer");

  Eigen::VectorXd noise = Eigen::VectorXd::Zero(graph_utils::getDim<T>());
  static const gtsam::SharedNoiseModel& init_noise = 
      gtsam::noiseModel::Diagonal::Sigmas(noise);

  gtsam::Key current_key = nfg[0]->front();

  gtsam::Values init_values; // add first value with prior factor 
  gtsam::NonlinearFactorGraph init_factors; 
  init_values.insert(current_key, values.at<T>(current_key));
  gtsam::PriorFactor<T> prior_factor(current_key, 
      values.at<T>(current_key), init_noise);

  pgo->loadGraph(nfg, values, prior_factor);

  // in case no loop closure, need to force optimize with only odom 
  pgo->force_optimize(); 
}
int main(int argc, char *argv[]) {
  gtsam::GraphAndValues graphNValues; 
  std::string dim = argv[1];
  std::string output_folder;
  if (argc > 5) output_folder = argv[5];
  bool verbose = false;

  if (argc > 6) {
    std::string flag = argv[6];
    if (flag == "v") verbose = true; 
  }

	if (dim == "2d") {
		graphNValues = gtsam::load2D(argv[2], gtsam::SharedNoiseModel(),
        0, false, true, gtsam::NoiseFormatG2O);
		Simulate<gtsam::Pose2>(graphNValues,
				atof(argv[3]), atof(argv[4]),
        output_folder, verbose);

  } else if (dim == "3d") {
    graphNValues = gtsam::load3D(argv[2]);
    Simulate<gtsam::Pose3>(graphNValues,
        atof(argv[3]), atof(argv[4]),
        output_folder, verbose);

  } else {
    log<WARNING>("Unsupported input format: ");
    log<WARNING>("Should be ./RpgoReadG2o <2d or 3d> <g2o file> <odom thresh> <pcm thresh> <opt: output_folder> <opt: v for messages");
  }
  
}