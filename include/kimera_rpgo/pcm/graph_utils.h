#pragma once

#include <Eigen/Dense>
#include <map>
#include <unordered_map>
#include <vector>

#include "kimera_rpgo/pcm/geometry_utils.h"

namespace kimera_rpgo {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>& max_clique);

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix,
                     std::vector<int>& max_clique);

int findMaxCliqueHeuIncremental(const Eigen::MatrixXd adjMatrix,
                                size_t num_new_lc,
                                size_t prev_maxclique_size,
                                std::vector<int>& max_clique);

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 *  This helps support having multiple robots (centralized, however)
 */
struct Trajectory {
  std::map<gtsam::Key, std::unique_ptr<Pose> > poses;

  /** \brief Get transform (along with node number and covariance)
   *  between two keys in trajectory
   *  from key_a to key_b
   */
  std::unique_ptr<Pose> getBetween(const gtsam::Key& key_a,
                                   const gtsam::Key& key_b) const;

  inline gtsam::Key getStartKey() const { return poses.begin()->first; }
};

}  // namespace kimera_rpgo
