/* 
Generic solver class 
author: Yun Chang
*/

#include "RobustPGO/RobustPGO.h"
#include "RobustPGO/pcm/pcm.h"

#include <gtsam/slam/dataset.h>

int main(int argc, char *argv[]) {
  const gtsam::GraphAndValues gv = gtsam::load3D(argv[1]);
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  OutlierRemoval *pcm = new PCM(1.0, 1.0); // with debug set to true
  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm)); // with debug set to true

  // first fix the first with a prior factor 
  gtsam::Key current_key = nfg[0]->front();
  gtsam::Vector6 noise;
  noise.setConstant(10.0);
	static const gtsam::SharedNoiseModel& init_noise = 
			gtsam::noiseModel::Diagonal::Sigmas(noise);

	gtsam::Values init_values; // add first value with prior factor 
	gtsam::NonlinearFactorGraph init_factors; 
	init_values.insert(current_key, values.at<gtsam::Pose3>(current_key));
	init_factors.add(gtsam::PriorFactor<gtsam::Pose3>(current_key, 
			values.at<gtsam::Pose3>(current_key), init_noise));
	pgo->update(init_factors, init_values);

	// For now assume that there are only odometry and loop closures 
  size_t num_factors = nfg.size(); 
  for (size_t i = 0; i < num_factors; i++) { 
  	gtsam::Key front = nfg[i]->front();
  	gtsam::Key back = nfg[i]->back();
  	if (front == current_key && front == back - 1) { 
  		// odometry factor 
      gtsam::Values new_values; 
      new_values.insert(back, values.at<gtsam::Pose3>(back));
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
}