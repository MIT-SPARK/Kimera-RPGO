// Authors: Yun Chang

#ifndef TYPE_UTILS_H
#define TYPE_UTILS_H

#include "RobustPGO/logger.h"

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace RobustPGO {

struct LandmarkMeasurements {
	gtsam::NonlinearFactorGraph factors; 
	gtsam::NonlinearFactorGraph consistent_factors; 
	gtsam::Matrix adj_matrix;
	gtsam::Matrix dist_matrix;

	LandmarkMeasurements(gtsam::NonlinearFactorGraph new_factors=gtsam::NonlinearFactorGraph()) :
			factors(new_factors),
			consistent_factors(new_factors) {
		if (new_factors.size() > 1) {
			log<WARNING>("Unexpected behavior: initializing landmark with more than one factor.");
		}
		adj_matrix = Eigen::MatrixXd::Zero(1,1);
		dist_matrix = Eigen::MatrixXd::Zero(1,1);
	}
};

}

#endif