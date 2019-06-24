// Authors: Pierre-Yves Lajoie, Yun Chang

#include "graph_utils.h"
#include "RobustPGO/logger.h"
#include "RobustPGO/max_clique_finder/findClique.h"

#include <fstream>
#include <iostream>
#include <math.h>

namespace graph_utils {

template <class T>
PoseWithCovariance<T> PoseWithCovariance<T>::compose(
    const PoseWithCovariance<T> other) const {
  PoseWithCovariance<T> out; 
  gtsam::Matrix Ha, Hb;
  out.pose = pose.pose.compose(other.pose, Ha, Hb);

  gtsam::Matrix tau1 = pose.pose.AdjointMap();

  out.covariance_matrix = pose.covariance_matrix +
      tau1 * other.covariance_matrix * tau1.transpose();

  return out;
}

template <class T>
PoseWithCovariance<T> PoseWithCovariance<T>::inverse() const {
  gtsam::Matrix Ha;
  PoseWithCovariance<T> out;
  out.pose = pose.pose.inverse(Ha);
  out.covariance_matrix = Ha * pose.covariance_matrix * Ha.transpose();

  return out;
}

template <class T>
PoseWithCovariance<T> PoseWithCovariance<T>::between(
    const PoseWithCovariance<T> other) const {

  PoseWithCovariance<T> out; 
  gtsam::Matrix Ha, Hb;
  out.pose = pose.pose.between(other.pose, Ha, Hb); // returns between in a frame 

  if (pose.pose.equals(other.pose)) {
    out.covariance_matrix = 
      Eigen::MatrixXd::Zero(pose.pose.getDimension(),pose.pose.getDimension());
    return out;
  }

  gtsam::Matrix tau1 = pose.pose.AdjointMap();

  out.covariance_matrix = tau1.inverse() * 
      (other.covariance_matrix - pose.covariance_matrix) * 
      tau1.transpose().inverse();

  bool PSD = true;
  Eigen::LLT<Eigen::MatrixXd> lltCovar1(out.covariance_matrix); // compute the Cholesky decomp
  if(lltCovar1.info() == Eigen::NumericalIssue){  
    PSD = false;
  } 

  if (!PSD) { 
    tau1 = other.pose.inverse().AdjointMap();
    out.covariance_matrix = tau1.inverse() * 
    (pose.covariance_matrix - other.covariance_matrix) * 
    tau1.transpose().inverse();

    Eigen::LLT<Eigen::MatrixXd> lltCovar2(out.covariance_matrix); // compute the Cholesky decomp
    if(lltCovar2.info() == Eigen::NumericalIssue){ 
      log<WARNING>("Warning: Covariance matrix between two poses not PSD"); 
    } 
  }
  return out;
}

int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix, 0.0);
  int max_clique_size = 0;
  max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique);
  return max_clique_size;
}

}