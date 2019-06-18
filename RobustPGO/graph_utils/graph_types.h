// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

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
struct PoseWithCovariance {
    gtsam::Pose3 pose;
    gtsam::Matrix covariance_matrix;
};

/** \struct Transform
 *  \brief Structure defining a transformation between two poses
 */
struct Transform {
    gtsam::Key i, j;
    graph_utils::PoseWithCovariance pose;
    bool is_separator;
};

/** \struct Transforms
 *  \brief Structure defining a std::map of transformations
 */
struct Transforms {
    gtsam::Key start_id, end_id;
    std::map<std::pair<gtsam::Key,gtsam::Key>, graph_utils::Transform> transforms;
};

/** \struct TrajectoryPose
 *  \brief Structure defining a pose in a robot trajectory
 */
struct TrajectoryPose {
    gtsam::Key id;
    graph_utils::PoseWithCovariance pose;
};

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 */
struct Trajectory {
    gtsam::Key start_id, end_id;
    std::map<gtsam::Key, graph_utils::TrajectoryPose> trajectory_poses;
};

/** \typedef LoopClosures
 *  \brief type to store poses IDs of loop closures.
 */
/** Type defining a list of pair of poses with a loop closure */
typedef std::vector<std::pair<gtsam::Key,gtsam::Key>> LoopClosures;

/** \typedef ConsistencyErrorData
 *  \brief type to store pose error vector and its associated covariance.
 */
/** Type defining a pair vector-matrix */
typedef std::pair<gtsam::Vector6, gtsam::Matrix> ConsistencyErrorData;

/** \var FIXED_COVARIANCE
 * \brief Covariance matrix with usual value (rotation std: 0.01 rad, translation std: 0.1 m).
 *
 * This value should not be used if covariance data is provided by the front end.
 */
const gtsam::Matrix FIXED_COVARIANCE =
        (Eigen::MatrixXd(6,6)  <<   0.0001, 0,      0,      0,     0,     0,
                                    0,      0.0001, 0,      0,     0,     0,
                                    0,      0,      0.0001, 0,     0,     0,
                                    0,      0,      0,      0.01,  0,     0,
                                    0,      0,      0,      0,     0.01,  0,
                                    0,      0,      0,      0,     0,     0.01).finished();

}
#endif