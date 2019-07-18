/* 
Outlier removal class 
Provide a set of outlier removal methods 
along with interface to robustPGO
author: Yun Chang
*/

#ifndef OUTLIERREMOVAL_H
#define OUTLIERREMOVAL_H

class OutlierRemoval {
public:
	virtual ~OutlierRemoval() = default;
	virtual bool process(gtsam::NonlinearFactorGraph new_factors,
				               gtsam::Values new_values,
				               gtsam::NonlinearFactorGraph& nfg,
				               gtsam::Values& values) = 0;

	virtual bool processForcedLoopclosure(
			gtsam::NonlinearFactorGraph new_factors,
			gtsam::Values new_values,
			gtsam::NonlinearFactorGraph& nfg,
			gtsam::Values& values) = 0; // force a loop closure

	virtual void saveData() {}

	void setQuiet() { debug_ = false; }

protected:
	bool debug_ = true;
};

#endif
