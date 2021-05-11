// Authors: Yun Chang

#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <Eigen/Dense>

namespace KimeraRPGO {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>* max_clique);

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix,
                     std::vector<int>* max_clique);

int findMaxCliqueHeuIncremental(const Eigen::MatrixXd adjMatrix,
                                size_t num_new_lc,
                                size_t prev_maxclique_size,
                                std::vector<int>* max_clique);

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 *  This helps support having multiple robots (centralized, however)
 */
template <class poseT, template <class> class T>
struct Trajectory {
  std::map<gtsam::Key, T<poseT> > poses;

  /** \brief Get transform (along with node number and covariance)
   *  between two keys in trajectory
   *  from key_a to key_b
   */
  T<poseT> getBetween(const gtsam::Key& key_a, const gtsam::Key& key_b) {
    gtsam::Symbol symb_key_a(key_a);
    gtsam::Symbol symb_key_b(key_b);
    if (symb_key_a.chr() == symb_key_b.chr()) {
      // same prefix: on same robot trajectory
      return poses[key_a].between(poses[key_b]);
    } else {
      char prefix_a = symb_key_a.chr();
      char prefix_b = symb_key_b.chr();
      // define root key
      gtsam::Key a0 = gtsam::Symbol(prefix_a, 0);
      gtsam::Key b0 = gtsam::Symbol(prefix_b, 0);
      T<poseT> pose_a = poses[a0].between(poses[key_a]);
      T<poseT> pose_b = poses[b0].between(poses[key_b]);
      T<poseT> pose_a0b0 = poses[a0].between(poses[b0]);

      // so now want a to b
      T<poseT> result = pose_a.inverse().compose(pose_a0b0);
      result = result.compose(pose_b);
      return result;
    }
  }

  inline gtsam::Key getStartKey() { return poses.begin()->first; }
};

}  // namespace KimeraRPGO