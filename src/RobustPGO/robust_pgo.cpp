/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include <RobustPGO/robust_pgo.h>

RobustPGO::RobustPGO(int solvertype): 
  nfg_(gtsam::NonlinearFactorGraph()),
  values_(gtsam::Values()),
  solver_type_(solvertype),
  nfg_odom_(gtsam::NonlinearFactorGraph()),
  nfg_lc_(gtsam::NonlinearFactorGraph()) {
  std::cout << "instantiated generic solver." << std::endl; 
}

bool RobustPGO::LoadParameters() {
  odom_threshold_ = 1.3; 
  pw_threshold_ = 1.3;
}


void RobustPGO::regularUpdate(gtsam::NonlinearFactorGraph nfg, 
                           gtsam::Values values, 
                           gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  // add new values and factors
  nfg_.add(nfg);
  values_.insert(values);
  bool do_optimize = true; 

  // print number of loop closures
  // std::cout << "number of loop closures so far: " << nfg_.size() - values_.size() << std::endl; 

  if (values.size() > 1) {log<WARNING>(L"Unexpected behavior: number of update poses greater than one.");}

  if (nfg.size() > 1) {log<WARNING>(L"Unexpected behavior: number of update factors greater than one.");}

  if (nfg.size() == 0 && values.size() > 0) {log<WARNING>(L"Unexpected behavior: added values but no factors.");}

  // Do not optimize for just odometry additions 
  // odometry values would not have prefix 'l' unlike artifact values
  if (nfg.size() == 1 && values.size() == 1) {
    const gtsam::Symbol symb(values.keys()[0]); 
    if (symb.chr() != 'l') {do_optimize = false;}
  }

  // nothing added so no optimization 
  if (nfg.size() == 0 && values.size() == 0) {do_optimize = false;}

  if (factorsToRemove.size() > 0) 
    do_optimize = true;

  if (do_optimize) {
    log<INFO>(L">>>>>>>>>>>> Run Optimizer <<<<<<<<<<<<");
    // optimize
    if (solver_type_ == 1) {
      gtsam::LevenbergMarquardtParams params;
      params.setVerbosityLM("SUMMARY");
      std::cout << "Running LM" << std::endl; 
      params.diagonalDamping = true; 
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 2) {
      gtsam::GaussNewtonParams params;
      params.setVerbosity("ERROR");
      std::cout << "Running GN" << std::endl; 
      values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 3) {
      // something
    }
  }
}

void RobustPGO::initializePrior(gtsam::PriorFactor<gtsam::Pose3> prior_factor) {
  gtsam::Pose3 initial_value = prior_factor.prior();
  gtsam::Matrix covar = Eigen::MatrixXd::Zero(6,6); // initialize as zero
  gtsam::Key initial_key = prior_factor.front(); // CHECK if correct 

  // construct initial pose with covar 
  graph_utils::PoseWithCovariance initial_pose; 
  initial_pose.pose = initial_value;
  initial_pose.covariance_matrix = covar; 
  graph_utils::TrajectoryPose init_trajpose; 
  init_trajpose.pose = initial_pose; 
  init_trajpose.id = initial_key;

  // populate posesAndCovariances_odom_
  posesAndCovariances_odom_.trajectory_poses[initial_key].pose = initial_pose;
  posesAndCovariances_odom_.start_id = initial_key;
  posesAndCovariances_odom_.end_id = initial_key;
}

void RobustPGO::updateOdom(gtsam::BetweenFactor<gtsam::Pose3> odom_factor, 
                               graph_utils::PoseWithCovariance &new_pose){

  // update posesAndCovariances_odom_ (compose last value with new odom value)
  
  // first get measurement and covariance and key from factor
  gtsam::Pose3 delta = odom_factor.measured(); 
  gtsam::Matrix covar =
      gtsam::inverse(boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>
      (odom_factor.get_noiseModel())->R()); // return covariance matrix
  gtsam::Key new_key = odom_factor.back();

  // construct pose with covariance for odometry measurement 
  graph_utils::PoseWithCovariance odom_delta; 
  odom_delta.pose = delta; 
  odom_delta.covariance_matrix = covar; 

  // Now get the latest pose in trajectory and compose 
  gtsam::Key latest_key = posesAndCovariances_odom_.end_id; 
  graph_utils::PoseWithCovariance last_pose = 
      posesAndCovariances_odom_.trajectory_poses.at(latest_key).pose; 
  // compose latest pose to odometry for new pose
  graph_utils::poseCompose(last_pose, odom_delta, new_pose);

  // update trajectory 
  posesAndCovariances_odom_.end_id = new_key; // update end key 
  // add to trajectory 
  graph_utils::TrajectoryPose new_trajectorypose; 
  new_trajectorypose.pose = new_pose;
  new_trajectorypose.id = new_key;
  posesAndCovariances_odom_.trajectory_poses[new_key] = new_trajectorypose; 
}

bool RobustPGO::isOdomConsistent(gtsam::BetweenFactor<gtsam::Pose3> lc_factor) {
  // assume loop is between pose i and j
  // extract the keys 
  gtsam::Key key_i = lc_factor.front();
  gtsam::Key key_j = lc_factor.back();
  
  graph_utils::PoseWithCovariance pij_odom, pji_lc, result;

  // access (T_i,Cov_i) and (T_j, Cov_j) from trajectory_
  graph_utils::PoseWithCovariance pi_odom, pj_odom; 
  pi_odom = posesAndCovariances_odom_.trajectory_poses[key_i].pose;
  pj_odom = posesAndCovariances_odom_.trajectory_poses[key_j].pose;

  graph_utils::poseBetween(pi_odom, pj_odom, pij_odom);

  // get pij_lc = (Tij_lc, Covij_lc) from factor
  pji_lc.pose = lc_factor.measured().inverse(); 
  gtsam::Matrix R_lc = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>
      (lc_factor.get_noiseModel())->R();
  
  // Check if includes rotation info 
  bool rotation_info = true; 
  if (R_lc.block<3,3>(0,0) == Eigen::MatrixXd::Zero(3,3)) {
    rotation_info = false; 
    R_lc.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * 0.0001;
  }

  pji_lc.covariance_matrix = gtsam::inverse(R_lc); // return covariance matrix

  // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
  graph_utils::poseCompose(pij_odom, pji_lc, result);
  result.pose.print("odom consistency check: ");
  std::cout << std::endl; 
  gtsam::Vector6 consistency_error = gtsam::Pose3::Logmap(result.pose);
  // check with threshold
  double threshold = odom_threshold_;
  // comput sqaure mahalanobis distance (the computation is wrong in robust mapper repo)
  double distance; 
  if (rotation_info) {
    distance = std::sqrt(consistency_error.transpose() 
        * gtsam::inverse(result.covariance_matrix) * consistency_error);
  } else {
    distance = std::sqrt(consistency_error.tail(3).transpose() 
        * gtsam::inverse(result.covariance_matrix.block<3,3>(3,3)) 
        * consistency_error.tail(3));
  }
  // TODO: print the mahalanobis dist of the loops in matrix
  log<INFO>(L"odometry consistency distance: %1%") % distance; 
  if (distance < threshold) {
    return true;
  }
  
  return false;
}

bool RobustPGO::areLoopsConsistent(gtsam::BetweenFactor<gtsam::Pose3> lc_1, 
                                     gtsam::BetweenFactor<gtsam::Pose3> lc_2) {
  // check if two loop closures are consistent 
  gtsam::Key key1a = lc_1.front();
  gtsam::Key key1b = lc_1.back();
  gtsam::Key key2a = lc_2.front();
  gtsam::Key key2b = lc_2.back();

  bool rotation_info = true; 

  graph_utils::PoseWithCovariance p1_lc_inv, p2_lc; 
  p1_lc_inv.pose = lc_1.measured().inverse();
  gtsam::Matrix R1_lc = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>
      (lc_1.get_noiseModel())->R();

  if (R1_lc.block<3,3>(0,0) == Eigen::MatrixXd::Zero(3,3)) {
    rotation_info = false; 
    R1_lc.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * 0.0001;
  }

  p1_lc_inv.covariance_matrix = gtsam::inverse(R1_lc); 

  p2_lc.pose = lc_2.measured();
  gtsam::Matrix R2_lc = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>
      (lc_2.get_noiseModel())->R();

  if (R2_lc.block<3,3>(0,0) == Eigen::MatrixXd::Zero(3,3)) {
    rotation_info = false; 
    R2_lc.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * 0.0001;
  }

  p2_lc.covariance_matrix = gtsam::inverse(R2_lc); 

  // find odometry from 1a to 2a 
  graph_utils::PoseWithCovariance p1a_odom, p2a_odom, p1a2a_odom; 
  p1a_odom = posesAndCovariances_odom_.trajectory_poses[key1a].pose;
  p2a_odom = posesAndCovariances_odom_.trajectory_poses[key2a].pose;
  graph_utils::poseBetween(p1a_odom, p2a_odom, p1a2a_odom);

  // find odometry from 2b to 1b 
  graph_utils::PoseWithCovariance p1b_odom, p2b_odom, p2b1b_odom; 
  p1b_odom = posesAndCovariances_odom_.trajectory_poses[key1b].pose;
  p2b_odom = posesAndCovariances_odom_.trajectory_poses[key2b].pose;
  graph_utils::poseBetween(p2b_odom, p1b_odom, p2b1b_odom);

  // check that lc_1 pose is consistent with pose from 1a to 1b 
  graph_utils::PoseWithCovariance p1a2b, p1a1b, result; 
  graph_utils::poseCompose(p1a2a_odom, p2_lc, p1a2b);
  graph_utils::poseCompose(p1a2b, p2b1b_odom, p1a1b);
  graph_utils::poseCompose(p1a1b, p1_lc_inv, result);
  // Might need to inverse p1_lc CHECK 

  gtsam::Vector6 consistency_error = gtsam::Pose3::Logmap(result.pose);
  // check with threshold
  double threshold = pw_threshold_; // hard coded for now 
  // comput sqaure mahalanobis distance 
  double distance; 
  if (rotation_info) {
    distance = std::sqrt(consistency_error.transpose() 
        * gtsam::inverse(result.covariance_matrix) * consistency_error);
  } else {
    distance = std::sqrt(consistency_error.tail(3).transpose() 
        * gtsam::inverse(result.covariance_matrix.block<3,3>(3,3)) 
        * consistency_error.tail(3));
  }

  log<INFO>(L"loop consistency distance: %1%") % distance; 
  if (distance < threshold) {
    return true;
  }

  return false;
}

void RobustPGO::findInliers(gtsam::NonlinearFactorGraph &inliers) {
  // * pairwise consistency check (will also compare other loops - if loop fails we still store it, but not include in the optimization)
  // -- add 1 row and 1 column to lc_adjacency_matrix_;
  // -- populate extra row and column by testing pairwise consistency of new lc against all previous ones
  // -- compute max clique
  // -- add loops in max clique to a local variable nfg_good_lc
  // NOTE: this will require a map from rowId (size_t, in adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
  size_t num_lc = nfg_lc_.size(); // number of loop closures so far
  Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
  if (num_lc > 1) {
    // if = 1 then just initialized 
    new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_adjacency_matrix_; 

    // now iterate through the previous loop closures and fill in last row + col 
    // of consistency matrix 
    for (size_t i = 0; i < num_lc - 1; i++) {
      gtsam::BetweenFactor<gtsam::Pose3> factor_i =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg_lc_[i]);
      gtsam::BetweenFactor<gtsam::Pose3> factor_j =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg_lc_[num_lc-1]);

      // check consistency 
      bool consistent = areLoopsConsistent(factor_i, factor_j);
      if (consistent) { 
        new_adj_matrix(num_lc-1, i) = 1; 
        new_adj_matrix(i, num_lc-1) = 1;
      }
    }
  }
  lc_adjacency_matrix_ = new_adj_matrix;
  std::cout << "adjacency matrix size: " 
      << lc_adjacency_matrix_.rows() << "," << lc_adjacency_matrix_.cols() << std::endl;

  std::vector<int> max_clique_data;
  int max_clique_size = graph_utils::findMaxClique(lc_adjacency_matrix_, max_clique_data);
  std::cout << "max clique size: " << max_clique_size << std::endl; 
  for (size_t i = 0; i < max_clique_size; i++) {
    std::cout << max_clique_data[i] << " "; 
    inliers.add(nfg_lc_[max_clique_data[i]]);
  }
}

void RobustPGO::update(gtsam::NonlinearFactorGraph nfg, 
                                 gtsam::Values values, 
                                 gtsam::FactorIndices factorsToRemove){
  // TODO: deal with factorsToRemove
  // check if odometry (compare factor keys) TODO: do we want to give odometry a specific prefix?
  // TODO: right now cant do between chordal factor 
  bool odometry = false; 
  bool loop_closure = false; 
  // test if odometry of loop closure (or neither in which case just do regular update)
  if (nfg.size() == 1 && values.size() == 1) {
    const gtsam::Symbol symb(values.keys()[0]); 
    if (symb.chr() != 'l') {
      boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > pose3Between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg[0]);
      if (pose3Between) {
        odometry = true;
      } else if (posesAndCovariances_odom_.trajectory_poses.size() == 0) {
        // probably a prior factor and initializing CHECK
        gtsam::PriorFactor<gtsam::Pose3> prior_factor =
            *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(nfg[0]);
        initializePrior(prior_factor);
        log<INFO>(L"Initialized prior and trajectory");
      }
    }
  } else if (nfg.size() == 1 && values.size() == 0){
    loop_closure = true; 
  }

  if (odometry) {
    // update posesAndCovariances_odom_;
    graph_utils::PoseWithCovariance new_pose;
    // extract between factor 
    gtsam::BetweenFactor<gtsam::Pose3> nfg_factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg[0]);

    updateOdom(nfg_factor, new_pose);
    // TODO: compare the new pose from out pose_compose with values pose 
    // should be the same 

    // - store factor in nfg_odom_
    nfg_odom_.add(nfg);

    // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)
    values_.insert(values.keys()[0], new_pose.pose);

    return;

  } else if (loop_closure) { // in this case we should run consistency check to see if loop closure is good
    // * odometric consistency check (will only compare against odometry - if loop fails this, we can just drop it)
    // extract between factor 
    gtsam::BetweenFactor<gtsam::Pose3> nfg_factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg[0]);

    if (isOdomConsistent(nfg_factor)) {
      nfg_lc_.add(nfg); // add factor to nfg_lc_

    } else {
      log<WARNING>(L"Discarded loop closure (inconsistent with odometry)");
      return; // discontinue since loop closure not consistent with odometry 
    }
    
    // Find inliers with Pairwise consistent measurement set maximization
    gtsam::NonlinearFactorGraph nfg_good_lc; 
    findInliers(nfg_good_lc);
    
    // * optimize and update values (for now just LM add others later)
    nfg_ = gtsam::NonlinearFactorGraph(); // reset 
    nfg_.add(nfg_odom_);
    nfg_.add(nfg_good_lc);
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    std::cout << "Running LM" << std::endl; 
    params.diagonalDamping = true; 
    values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    return; 

  } else {
    // NOTE this case 
    regularUpdate(nfg, values, factorsToRemove);
  }
}
