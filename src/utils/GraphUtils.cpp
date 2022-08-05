// Authors: Yun Chang
#include <vector>

#include "KimeraRPGO/max_clique_finder/findClique.h"
#include "KimeraRPGO/utils/GraphUtils.h"

namespace KimeraRPGO {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>* max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  size_t max_clique_size = 0;
  max_clique_size = FMC::maxClique(&gio, max_clique_size, max_clique);
  return max_clique_size;
}

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix,
                     std::vector<int>* max_clique) {
  // Compute maximum clique (heuristic inexact version)
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  int max_clique_size = 0;
  max_clique_size = FMC::maxCliqueHeu(&gio, max_clique);
  return max_clique_size;
}

// TODO
int findMaxCliqueHeuIncremental(const Eigen::MatrixXd adjMatrix,
                                size_t num_new_lc,
                                size_t prev_maxclique_size,
                                std::vector<int>* max_clique) {
  // Compute maximum clique (heuristic inexact version)
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  int max_clique_size_new_lc = 0;
  max_clique_size_new_lc = FMC::maxCliqueHeuIncremental(
      &gio, num_new_lc, prev_maxclique_size, max_clique);
  if (static_cast<size_t>(max_clique_size_new_lc) > prev_maxclique_size) {
    return max_clique_size_new_lc;
  }
  return 0;
}

}  // namespace KimeraRPGO
