/*
Outlier removal class
Provide a set of outlier removal methods
along with interface to robustPGO
author: Yun Chang
*/

#ifndef OUTLIERREMOVAL_H
#define OUTLIERREMOVAL_H

namespace RobustPGO {

class OutlierRemoval {
public:
	virtual ~OutlierRemoval() = default;
	// TODO(Luca): this must be called removeOutliers
	virtual bool process(const gtsam::NonlinearFactorGraph& new_factors,
				               const gtsam::Values& new_values,
				               gtsam::NonlinearFactorGraph& nfg,
				               gtsam::Values& values) = 0;

	// need comments here
	virtual bool processForcedLoopclosure(
			const gtsam::NonlinearFactorGraph& new_factors,
			const gtsam::Values& new_values,
			gtsam::NonlinearFactorGraph& nfg,
			gtsam::Values& values) = 0; // force a loop closure

	virtual void saveData(std::string folder_path) {}

	void setQuiet() { debug_ = false; }

protected:
	bool debug_ = true;
};

}
#endif
