/* 
Robust Solver Params class 
author: Yun Chang
*/

namespace RobustPGO {

enum class Solver { LM, GN };

enum class OutlierRemovalMethod {
	PCM, 
	PCM_Distance
};

struct RobustSolverParams {
public:
	RobustSolverParams():
			solver(LM),
			outlierRemovalMethod(PCM),
			pcm_lcThreshold(5.0),
			pcm_odomThreshold(10.0),
			specialSymbols() {}

	// General
	Solver solver;
	OutlierRemovalMethod outlierRemovalMethod;
	std::vector<char> special_symbols;

	// for PCM
	double pcm_lcThreshold; 
	double pcm_odomThreshold;

	// for PCM_Distance
	double pcmDist_transThreshold;
	double pcmDist_rotThreshold;
};

}