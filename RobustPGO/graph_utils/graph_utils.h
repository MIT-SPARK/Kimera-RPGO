// Authors: Pierre-Yves Lajoie, Yun Chang

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <map>
#include <vector>

/** \namespace graph_utils
 *  \brief This namespace encapsulates utility functions to manipulate graphs
 */
namespace graph_utils {

/** \struct PoseWithCovariance
 *  \brief Structure to store a pose and its covariance data
 */
template <class T>
struct PoseWithCovariance {
    T pose; // ex. gtsam::Pose3
    gtsam::Matrix covariance_matrix;

    PoseWithCovariance compose(const PoseWithCovariance other) const;
    PoseWithCovariance inverse() const;
    PoseWithCovariance between(const PoseWithCovariance other) const;
};

/** \struct Transform
 *  \brief Structure defining a transformation between two poses
 */
template <class T>
struct Transform {
    gtsam::Key i, j;
    graph_utils::PoseWithCovariance<T> pose;
    bool is_separator;
};

/** \struct Transforms
 *  \brief Structure defining a std::map of transformations
 */
template <class T>
struct Transforms {
    gtsam::Key start_id, end_id;
    std::map<std::pair<gtsam::Key,gtsam::Key>, graph_utils::Transform<T>> transforms;
};

/** \struct TrajectoryPose
 *  \brief Structure defining a pose in a robot trajectory
 */
template <class T>
struct TrajectoryPose {
    gtsam::Key id;
    graph_utils::PoseWithCovariance<T> pose;
};

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 */
template <class T>
struct Trajectory {
    gtsam::Key start_id, end_id;
    std::map<gtsam::Key, graph_utils::TrajectoryPose<T>> trajectory_poses;
};

int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique);

}
#endif