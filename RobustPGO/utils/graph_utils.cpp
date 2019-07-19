// Authors: Yun Chang

#include "RobustPGO/utils/graph_utils.h"

namespace RobustPGO {

int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix, 0.0);
  int max_clique_size = 0;
  max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique);
  return max_clique_size;
}

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique) {
  // Compute maximum clique (heuristic inexact version)
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix, 0.0);
  int max_clique_size = 0;
  max_clique_size = FMC::maxCliqueHeu(gio, max_clique);
  return max_clique_size;
}

}