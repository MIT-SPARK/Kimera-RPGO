// Authors: Yun Chang

#include "KimeraRPGO/utils/graph_utils.h"
#include "KimeraRPGO/max_clique_finder/findClique.h"

namespace KimeraRPGO {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>& max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  int max_clique_size;
  max_clique_size = FMC::maxClique(&gio, max_clique_size, &max_clique);
  return max_clique_size;
}

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix,
                     std::vector<int>& max_clique) {
  // Compute maximum clique (heuristic inexact version)
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  int max_clique_size = 0;
  max_clique_size = FMC::maxCliqueHeu(&gio, &max_clique);
  return max_clique_size;
}

}  // namespace KimeraRPGO
