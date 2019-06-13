// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_FUNCTIONS_H
#define GRAPH_UTILS_FUNCTIONS_H

#include "graph_types.h"
#include "RobustPGO/max_clique_finder/findClique.h"
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>

namespace graph_utils {
/** \brief This function parse .g2o files.
 *
 * The specification of this format is available here : https://github.com/RainerKuemmerle/g2o/wiki/File-Format
 * @param[in] file_name File name
 * @param[out] num_poses Number of poses in the file
 * @param[out] transforms Structure containing the measurements
 * @param[out] loop_closures IDs of the nodes involved to loop closures
 * @param[in] only_loop_closures If true, the file is expected to contain only loop closures
 * @return the degrees of freedom (3 in 2D, 6 in 3D)
 */
uint8_t parseG2ofile(const std::string &file_name, size_t &num_poses, 
    Transforms& tranforms,
    LoopClosures& loop_closures, 
    const bool& only_loop_closures);

/** \brief This function compose (+) geometric poses with covariance.
 *
 * @param[in] a,b Geometric poses with covariance information
 * @param[out] out is a + b
 */
void poseCompose(const graph_utils::PoseWithCovariance &a,
                 const graph_utils::PoseWithCovariance &b,
                 graph_utils::PoseWithCovariance &out);

/** \brief This function invert a geometric pose with covariance.
 *
 * @param[in] a Geometric poses with covariance information
 * @param[out] out is I - a
 */
void poseInverse(const graph_utils::PoseWithCovariance &a,
                 graph_utils::PoseWithCovariance &out);

/** \brief This function between (-) geometric poses with covariance.
 *
 * @param[in] a,b Geometric poses with covariance information
 * @param[out] out is a - b
 */
void poseBetween(const graph_utils::PoseWithCovariance &a,
                 const graph_utils::PoseWithCovariance &b,
                 graph_utils::PoseWithCovariance &out);

/** \brief This function precompute the trajectory by composing the successive poses.
 *
 * @param[in] transforms Odometry measurements
 * returns the trajectory
 */
Trajectory buildTrajectory(const Transforms& transforms);

/** \brief This function prints the consistency matrix to the format expected by the maximum clique solver
 *
 * This function prints the consistency matrix to the format expected by the maximum clique solver
 * Fast Max-Cliquer (http://cucis.ece.northwestern.edu/projects/MAXCLIQUE/)
 * @param[in] consistency_matrix Consistency matrix of the loop closures
 * @param[in] file_name Name of the file in which the consistency matrix will be saved
 */
void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix, const std::string& file_name);

/** \brief This function check if a pose is include in a trajectory.
 *
 * @param[in] trajectory Trajectory in which to find the pose
 * @param[in] pose_id ID of the pose to find.
 * returns boolean indicating if the pose is included in the trajectory.
*/
bool isInTrajectory(const Trajectory& trajectory, const size_t& pose_id);

/** \brief This function prints a list of consistent loop closures in a file.
*
* @param[in] loop_closures Loop closures to consider
* @param[in] max_clique_data Max-Cliquer output data.
* @param[in] file_name Name of the file to save the results.
*/
void printConsistentLoopClosures(const LoopClosures& loop_closures, const std::vector<int>& max_clique_data, const std::string& file_name);

int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique);

}

#endif