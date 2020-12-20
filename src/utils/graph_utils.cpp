// Authors: Yun Chang
#include <vector>
#include "pmc/pmc.h"

#include "KimeraRPGO/max_clique_finder/findClique.h"
#include "KimeraRPGO/utils/graph_utils.h"

namespace KimeraRPGO {

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
  if (max_clique_size_new_lc > prev_maxclique_size) {
    return max_clique_size_new_lc;
  }
  return 0;
}

int findMaxClique(const Eigen::MatrixXd& adjMatrix,
                  std::vector<int>* max_clique) {
  // First convert adjacency matrix to PMC graph
  std::vector<int> edges;
  std::vector<long long> vertices;
  vertices.push_back(edges.size());

  if (adjMatrix.rows() != adjMatrix.cols()) {
    std::cout << "ERROR: expected adjacency matrix to be symmetric. "
              << std::endl;
    return 0;
  }

  for (size_t i = 0; i < adjMatrix.rows(); i++) {
    for (size_t j = 0; j < adjMatrix.cols(); j++) {
      if (adjMatrix(i, j) > 0) {
        edges.push_back(j);
      }
    }
    vertices.push_back(edges.size());
  }

  pmc::pmc_graph G(vertices, edges);

  // Prepare PMC input
  pmc::input in;
  in.algorithm = 0;
  in.threads = 12;
  in.experiment = 0;
  in.lb = 0;
  in.ub = 0;
  in.param_ub = 0;
  in.adj_limit = 20000;
  in.time_limit = 3600;
  in.remove_time = 4;
  in.graph_stats = false;
  in.verbose = false;
  in.help = false;
  in.MCE = false;
  in.decreasing_order = false;
  in.heu_strat = "kcore";
  in.vertex_search_order = "deg";

  // Exact max clique finding
  // The following methods are used:
  // 1. k-core pruning
  // 2. neigh-core pruning/ordering
  // 3. dynamic coloring bounds/sort
  // see the original PMC paper and implementation for details:
  // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum
  // Clique Algorithms with Applications to Network Analysis,” SIAM J. Sci.
  // Comput., vol. 37, no. 5, pp. C589–C616, Jan. 2015.
  if (G.num_vertices() < in.adj_limit) {
    G.create_adj();
    pmc::pmcx_maxclique finder(G, in);
    finder.search_dense(G, *max_clique);
  } else {
    pmc::pmcx_maxclique finder(G, in);
    finder.search(G, *max_clique);
  }

  return max_clique->size();
}

}  // namespace KimeraRPGO
