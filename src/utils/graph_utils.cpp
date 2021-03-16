// Authors: Yun Chang
#include <vector>
#include "pmc/pmc.h"

#include "KimeraRPGO/max_clique_finder/findClique.h"
#include "KimeraRPGO/utils/graph_utils.h"

#include "clipper/clipper.h"
#include "clipper/find_dense_cluster.h"

namespace KimeraRPGO {

pmc::pmc_graph adjMatrixToPmcGraph(const Eigen::MatrixXd& adjMatrix) {
  // First convert adjacency matrix to PMC graph
  std::vector<int> edges;
  std::vector<long long> vertices;
  vertices.push_back(edges.size());

  if (adjMatrix.rows() != adjMatrix.cols()) {
    std::cout << "ERROR: expected adjacency matrix to be symmetric. "
              << std::endl;
    return pmc::pmc_graph(vertices, edges);
  }

  for (size_t i = 0; i < adjMatrix.rows(); i++) {
    for (size_t j = 0; j < adjMatrix.cols(); j++) {
      if (adjMatrix(i, j) > 0) {
        edges.push_back(j);
      }
    }
    vertices.push_back(edges.size());
  }

  return pmc::pmc_graph(vertices, edges);
}

int findMaxCliqueHeu(const Eigen::MatrixXd& adjMatrix,
                     std::vector<int>* max_clique) {
  if (adjMatrix.rows() == 1) {
    max_clique->push_back(0);
    return 1;
  }

  pmc::pmc_graph G = adjMatrixToPmcGraph(adjMatrix);

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

  G.compute_cores();
  auto max_core = G.get_max_core();

  if (max_core > static_cast<int>(static_cast<double>(adjMatrix.rows()))) {
    auto k_cores = G.get_kcores();
    for (int i = 1; i < k_cores->size(); ++i) {
      // Note: k_core has size equals to num vertices + 1
      if ((*k_cores)[i] >= max_core) {
        max_clique->push_back(i - 1);
      }
    }
    return max_clique->size();
  }

  if (in.ub == 0) {
    in.ub = max_core + 1;
  }

  // lower-bound of max clique
  if (in.lb == 0 && in.heu_strat != "0") {  // skip if given as input
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, *max_clique);
  }

  assert(in.lb != 0);
  if (in.lb == 0) {
    // This means that max clique has a size of one
    max_clique->push_back(0);
    return max_clique->size();
  }

  if (in.lb == in.ub) {
    return max_clique->size();
  }

  return max_clique->size();
}

// TODO FIXME not implemented yet
int findMaxCliqueHeuIncremental(const Eigen::MatrixXd adjMatrix,
                                size_t num_new_lc,
                                size_t prev_maxclique_size,
                                std::vector<int>* max_clique) {
  // Compute maximum clique (heuristic inexact version)
  return 0;
}

int findMaxClique(const Eigen::MatrixXd& adjMatrix,
                  std::vector<int>* max_clique) {
  if (adjMatrix.rows() == 1) {
    max_clique->push_back(0);
    return 1;
  }

  pmc::pmc_graph G = adjMatrixToPmcGraph(adjMatrix);

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

  G.compute_cores();
  auto max_core = G.get_max_core();

  if (in.ub == 0) {
    in.ub = max_core + 1;
  }

  // lower-bound of max clique
  if (in.lb == 0 && in.heu_strat != "0") {  // skip if given as input
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, *max_clique);
  }

  assert(in.lb != 0);
  if (in.lb == 0) {
    // This means that max clique has a size of one
    max_clique->push_back(0);
    return max_clique->size();
  }

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

int findMaxCliqueClipper(const Eigen::MatrixXd& adjMatrix,
                     std::vector<int>* max_clique) {

  int dim = adjMatrix.rows();

  // clipper needs adjMatrix + I
  // perhaps it'd be more efficient if we add I when constructing ajdMatrix
  Eigen::MatrixXd adjMatrixPlusId = adjMatrix + Eigen::MatrixXd::Identity(dim,dim);

  clipper::Params params;
  clipper::Solution soln = clipper::findDenseCluster(adjMatrixPlusId, adjMatrixPlusId, params);

  *max_clique = soln.nodes;

  int clipper_max_clique_size = soln.nodes.size();

  std::cout << " |x| clipper = " << clipper_max_clique_size << std::endl;
  
  return clipper_max_clique_size;
}

}  // namespace KimeraRPGO
