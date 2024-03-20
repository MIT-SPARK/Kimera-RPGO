/*
Example file to perform robust optimization on g2o files
author: Yun Chang
*/
#include <memory>
#include <string>

#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/LoadG2o.h"

using namespace KimeraRPGO;

int main(int argc, char* argv[]) {
  // Refine for now test with victoria park

  bool is_3d = false;
  std::string g2ofile = argv[1];
  std::string output_folder = argv[2];
  double gnc_alpha = std::stod(argv[3]);

  const gtsam::GraphAndValues& graph_and_values =
      gtsam::readG2owithLmks(g2ofile, is_3d);

  RobustSolverParams params;
  params.logOutput(output_folder);
  Verbosity verbosity = Verbosity::VERBOSE;
  std::unique_ptr<GenericSolver> pgo;

  if (gnc_alpha <= 0 || gnc_alpha >= 1) {
    // No rejection
    params.setNoRejection(verbosity);
    pgo = KimeraRPGO::make_unique<GenericSolver>(Solver::LM);
  } else {
    params.setPcmSimple2DParams(-1, -1, verbosity);
    params.setGncInlierCostThresholdsAtProbability(gnc_alpha);
    pgo = KimeraRPGO::make_unique<RobustSolver>(params);
  }

  pgo->update(*graph_and_values.first, *graph_and_values.second);
  pgo->saveData(output_folder);
}
