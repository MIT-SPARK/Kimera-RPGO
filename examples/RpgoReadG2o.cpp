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

/* Usage: ./RpgoReadG2o 2d <some-2d-g2o-file> <odom-threshold> <pcm-threshold>
   [or]   ./RpgoReadG2o 3d <some-3d-g2o-file> <odom-threshold> <pcm-threshold>*/
template<class T>
void Simulate(gtsam::GraphAndValues gv, 
			double odom_thresh, double pmc_thresh) {

	gtsam::NonlinearFactorGraph nfg = *gv.first;

  gtsam::Values values = *gv.second;

  OutlierRemoval *pcm = new PCM<T>(odom_thresh, pmc_thresh);
  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));

  // first fix the first with a prior factor 
  gtsam::Key current_key = nfg[0]->front();
  Eigen::VectorXd noise = Eigen::VectorXd::Zero(graph_utils::getDim<T>());
	static const gtsam::SharedNoiseModel& init_noise = 
			gtsam::noiseModel::Diagonal::Sigmas(noise);

	gtsam::Values init_values; // add first value with prior factor 
	gtsam::NonlinearFactorGraph init_factors; 
	init_values.insert(current_key, values.at<T>(current_key));
	init_factors.add(gtsam::PriorFactor<T>(current_key, 
			values.at<T>(current_key), init_noise));
	pgo->update(init_factors, init_values);

	// For now assume that there are only odometry and loop closures 
  size_t num_factors = nfg.size(); 
  for (size_t i = 0; i < num_factors; i++) { 
  	gtsam::Key front = nfg[i]->front();
  	gtsam::Key back = nfg[i]->back();
  	if (front == current_key && front == back - 1) { 
  		// odometry factor 
      gtsam::Values new_values; 
      new_values.insert(back, values.at<T>(back));
      gtsam::NonlinearFactorGraph new_factors; 
      new_factors.add(nfg[i]);
      std::cout << "odometry: " << front << ">" << back << std::endl; 
      nfg[i]->print();
      pgo->update(new_factors, new_values);
      current_key++; 
    } else { 
  		// loop closure factor 
      gtsam::NonlinearFactorGraph new_factors; 
      new_factors.add(nfg[i]);
      std::cout << "loop closure: " << front << ">" << back << std::endl;
      nfg[i]->print(); 
      pgo->update(new_factors, gtsam::Values());
  	}
  }
  // in case no loop closure, need to force optimize with only odom 
  pgo->force_optimize(); 
}
int main(int argc, char *argv[]) {
	gtsam::GraphAndValues graphNValues; 
	std::string dim = argv[1];
	if (dim == "2d") {
		graphNValues = gtsam::load2D(argv[2]);
		Simulate<gtsam::Pose2>(graphNValues,
				atof(argv[3]), atof(argv[4]));

	} else if (dim == "3d") {
		graphNValues = gtsam::load3D(argv[2]);
		Simulate<gtsam::Pose3>(graphNValues,
				atof(argv[3]), atof(argv[4]));
	} else {
		log<WARNING>("Unsupported input format: ");
		log<WARNING>("Should be ./RpgoReadG2o <2d or 3d> <g2o file> <odom thresh> <pcm thresh>");
	}
  
}