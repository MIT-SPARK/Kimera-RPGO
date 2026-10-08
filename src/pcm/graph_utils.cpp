#include "kimera_rpgo/pcm/graph_utils.h"

#include <gtsam/inference/Symbol.h>

#include <vector>

#include "kimera_rpgo/pcm/max_clique_finder/findClique.h"

namespace kimera_rpgo {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>& max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  size_t max_clique_size = 0;
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

int findMaxCliqueHeuIncremental(const Eigen::MatrixXd adjMatrix,
                                size_t num_new_lc,
                                size_t prev_maxclique_size,
                                std::vector<int>& max_clique) {
  // Compute maximum clique (heuristic inexact version)
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix);
  int max_clique_size_new_lc = 0;
  max_clique_size_new_lc = FMC::maxCliqueHeuIncremental(
      &gio, num_new_lc, prev_maxclique_size, &max_clique);
  if (static_cast<size_t>(max_clique_size_new_lc) > prev_maxclique_size) {
    return max_clique_size_new_lc;
  }

  return 0;
}

std::unique_ptr<Pose> Trajectory::getBetween(const gtsam::Key& key_a,
                                             const gtsam::Key& key_b) const {
  gtsam::Symbol symb_key_a(key_a);
  gtsam::Symbol symb_key_b(key_b);
  if (symb_key_a.chr() == symb_key_b.chr()) {
    // same prefix: on same robot trajectory
    return poses.at(key_a)->between(poses.at(key_b));
  } else {
    char prefix_a = symb_key_a.chr();
    char prefix_b = symb_key_b.chr();
    // define root key
    gtsam::Key a0 = gtsam::Symbol(prefix_a, 0);
    gtsam::Key b0 = gtsam::Symbol(prefix_b, 0);
    auto pose_a = poses.at(a0)->between(poses.at(key_a));
    auto pose_b = poses.at(b0)->between(poses.at(key_b));
    auto pose_a0b0 = poses.at(a0)->between(poses.at(b0));

    // so now want a to b
    auto result = pose_a->inverse()->compose(pose_a0b0);
    return result->compose(pose_b);
  }
}
}  // namespace kimera_rpgo
