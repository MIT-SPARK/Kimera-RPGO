/*
Robust Solver Params class
author: Yun Chang
*/

#ifndef SOLVERPARAMS_H
#define SOLVERPARAMS_H

namespace RobustPGO {

enum class Solver { LM, GN };

// TODO(Luca): OutlierRemoval should not care about 2D or 3D
enum class OutlierRemovalMethod {
	NONE, // no outlier rejection
	PCM2D,
	PCM3D,
	PCM_Simple2D,
	PCM_Simple3D
};

enum class Verbosity {
	UPDATE,
	QUIET,
	VERBOSE
};

struct RobustSolverParams {
public:
	RobustSolverParams():
			solver(Solver::LM), // TODO(Luca): default should be GN (faster, but we can keep LM for subt)
			outlierRemovalMethod(OutlierRemovalMethod::PCM3D),
			specialSymbols(),
			verbosity(Verbosity::UPDATE),
			pcm_odomThreshold(10.0),
			pcm_lcThreshold(5.0),
			pcmDist_transThreshold(0.05), // 5cm
			pcmDist_rotThreshold(0.005){} // <0.5degrees

	void setNoRejection(Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::NONE;
		verbosity = verbos;
	}

	// TODO(Luca): no 2D & 3D version
	void setPcm2DParams(double odomThreshold,
			double lcThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM2D;
		pcm_odomThreshold = odomThreshold;
		pcm_lcThreshold = lcThreshold;
		verbosity = verbos;
	}

	void setPcm3DParams(double odomThreshold,
			double lcThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM3D;
		pcm_odomThreshold = odomThreshold;
		pcm_lcThreshold = lcThreshold;
		verbosity = verbos;
	}

	// TODO(Luca): no 2D & 3D version
	void setPcmSimple2DParams(double transThreshold,
			double rotThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple2D;
		pcmDist_transThreshold = transThreshold;
		pcmDist_rotThreshold = rotThreshold;
		verbosity = verbos;
	}

	void setPcmSimple3DParams(double transThreshold,
			double rotThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple3D;
		pcmDist_transThreshold = transThreshold;
		pcmDist_rotThreshold = rotThreshold;
		verbosity = verbos;
	}

	// General
	Solver solver;
	OutlierRemovalMethod outlierRemovalMethod;
	std::vector<char> specialSymbols;
	Verbosity verbosity;

	// for PCM
	double pcm_odomThreshold;
	double pcm_lcThreshold;

	// for PCM_Distance
	double pcmDist_transThreshold;
	double pcmDist_rotThreshold;
};

}
#endif
