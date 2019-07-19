/* 
Robust Solver Params class 
author: Yun Chang
*/

#ifndef SOLVERPARAMS_H
#define SOLVERPARAMS_H

namespace RobustPGO {

enum class Solver { LM, GN };

enum class OutlierRemovalMethod {
	PCM2D,
	PCM3D,
	PCM_Distance2D,
	PCM_Distance3D
};

enum class Verbosity {
	UPDATE, 
	QUIET,
	ERROR
};

struct RobustSolverParams {
public:
	RobustSolverParams():
			solver(Solver::LM),
			outlierRemovalMethod(OutlierRemovalMethod::PCM3D),
			specialSymbols(),
			verbosity(Verbosity::UPDATE),
			pcm_odomThreshold(10.0),
			pcm_lcThreshold(5.0) {}

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

	void setPcmDist2DParams(double transThreshold, 
			double rotThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM_Distance2D;
		pcmDist_transThreshold = transThreshold;
		pcmDist_rotThreshold = rotThreshold;
		verbosity = verbos;
	}

	void setPcmDist3DParams(double transThreshold, 
			double rotThreshold, Verbosity verbos=Verbosity::UPDATE) {
		outlierRemovalMethod = OutlierRemovalMethod::PCM_Distance3D;
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