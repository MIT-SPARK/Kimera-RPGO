/*
Robust Solver Params class
author: Yun Chang
*/

#pragma once

#include <string>
#include <vector>

namespace KimeraRPGO {

enum class Solver { LM, GN };

// TODO(Luca): OutlierRemoval should not care about 2D or 3D
enum class OutlierRemovalMethod {
  NONE,  // no outlier rejection
  PCM2D,
  PCM3D,
  PCM_Simple2D,
  PCM_Simple3D
};

enum class Verbosity { UPDATE, QUIET, VERBOSE };

// Method used for multi-robot frame alignment
enum class MultiRobotAlignMethod {
  NONE,  // Use provided initial guess
  L2,    // Use L2 pose averaging
  GNC    // Use robust pose averaging with GNC
};

struct PcmParams {
public:
 PcmParams()
     : odom_threshold(10.0),
       lc_threshold(5.0),
       odom_trans_threshold(0.05),
       odom_rot_threshold(0.005),
       dist_trans_threshold(0.01),
       dist_rot_threshold(0.001),
       incremental(false) {}
 // for Pcm
 double odom_threshold;
 double lc_threshold;

 // for PcmSimple
 double odom_trans_threshold;
 double odom_rot_threshold;
 double dist_trans_threshold;
 double dist_rot_threshold;

 // incremental max clique
 bool incremental;
};

struct RobustSolverParams {
 public:
  RobustSolverParams()
      : solver(Solver::LM),  // TODO(Luca): default should be GN (faster, but we
                             // can keep LM for subt)
        outlierRemovalMethod(OutlierRemovalMethod::PCM3D),
        specialSymbols(),
        verbosity(Verbosity::UPDATE),
        pcm_params(),
        log_output(false),
        use_gnc_(false),
        multirobot_align_method(MultiRobotAlignMethod::NONE) {}
  /*! \brief For RobustSolver to not do outlier rejection at all
   */
  void setNoRejection(Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::NONE;
    verbosity = verbos;
  }

  /*! \brief use incremental max clique
   */
  void setIncremental() { pcm_params.incremental = true; }

  /*! \brief 2D version of Pcm
   * This one looks at Mahalanobis distance
   * odomThreshold: max allowable M distance deviation from odometry
   * lcThreshold: max allowable M distance deviation between pairs of
   * measurements
   */
  void setPcm2DParams(double odomThreshold,
                      double lcThreshold,
                      Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM2D;
    pcm_params.odom_threshold = odomThreshold;
    pcm_params.lc_threshold = lcThreshold;
    verbosity = verbos;
  }

  /*! \brief 3D version of Pcm
   * This one looks at Mahalanobis distance
   * odomThreshold: max allowable M distance deviation from odometry
   * lcThreshold: max allowable M distance deviation between pairs of
   * measurements
   */
  void setPcm3DParams(double odomThreshold,
                      double lcThreshold,
                      Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM3D;
    pcm_params.odom_threshold = odomThreshold;
    pcm_params.lc_threshold = lcThreshold;
    verbosity = verbos;
  }

  /*! \brief 2D version of PcmSimple
   * This one looks at average translation and rotation error per node
   * transThreshold: Estimated max drift in translation per node (in meters)
   * rotThreshold: Estimated max drift in rotation per node (in radians)
   */
  void setPcmSimple2DParams(double transThreshold,
                            double rotThreshold,
                            Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple2D;
    pcm_params.odom_trans_threshold = transThreshold;
    pcm_params.odom_rot_threshold = rotThreshold;
    pcm_params.dist_trans_threshold = transThreshold;
    pcm_params.dist_rot_threshold = rotThreshold;
    verbosity = verbos;
  }

  /*! \brief 3D version of PcmSimple
   * This one looks at average translation and rotation error per node
   * transThreshold: Estimated max drift in translation per node (in meters)
   * rotThreshold: Estimated max drift in rotation per node (in radians)
   */
  void setPcmSimple3DParams(double transThreshold,
                            double rotThreshold,
                            Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple3D;
    pcm_params.odom_trans_threshold = transThreshold;
    pcm_params.odom_rot_threshold = rotThreshold;
    pcm_params.dist_trans_threshold = transThreshold;
    pcm_params.dist_rot_threshold = rotThreshold;
    verbosity = verbos;
  }

  /*! \brief 2D version of PcmSimple with separate parameters for odom check
   * This one looks at average translation and rotation error per node
   * transThreshold: Estimated max drift in translation per node (in meters)
   * rotThreshold: Estimated max drift in rotation per node (in radians)
   */
  void setPcmSimple2DParams(double transOdomThreshold,
                            double rotOdomThreshold,
                            double transPcmThreshold,
                            double rotPcmThreshold,
                            Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple2D;
    pcm_params.odom_trans_threshold = transOdomThreshold;
    pcm_params.odom_rot_threshold = rotOdomThreshold;
    pcm_params.dist_trans_threshold = transPcmThreshold;
    pcm_params.dist_rot_threshold = rotPcmThreshold;
    verbosity = verbos;
  }

  /*! \brief 3D version of PcmSimple with separate parameters for odom check
   * This one looks at average translation and rotation error per node
   * transThreshold: Estimated max drift in translation per node (in meters)
   * rotThreshold: Estimated max drift in rotation per node (in radians)
   */
  void setPcmSimple3DParams(double transOdomThreshold,
                            double rotOdomThreshold,
                            double transPcmThreshold,
                            double rotPcmThreshold,
                            Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::PCM_Simple3D;
    pcm_params.odom_trans_threshold = transOdomThreshold;
    pcm_params.odom_rot_threshold = rotOdomThreshold;
    pcm_params.dist_trans_threshold = transPcmThreshold;
    pcm_params.dist_rot_threshold = rotPcmThreshold;
    verbosity = verbos;
  }

  /*! \brief one way of setting GNC parameters (confidence threshold)
   */
  void setGncInlierCostThresholdsAtProbability(const double& alpha) {
    use_gnc_ = true;
    gnc_threshold_mode_ = GncThresholdMode::PROBABILITY;
    gnc_inlier_threshold_ = alpha;
  }

  /*! \brief one way of setting GNC parameters (cost threshold)
   */
  void setGncInlierCostThresholds(const double& cost) {
    use_gnc_ = true;
    gnc_threshold_mode_ = GncThresholdMode::COST;
    gnc_inlier_threshold_ = cost;
  }

  /*! \brief use multirobot frame alignment for initialization
   */
  void setMultiRobotAlignMethod(MultiRobotAlignMethod method) {
    multirobot_align_method = method;
  }

  /*! \brief set folder to log data
   */
  void logOutput(const std::string& output_folder) {
    log_output = true;
    log_folder = output_folder;
  }

  // General
  Solver solver;
  OutlierRemovalMethod outlierRemovalMethod;
  std::vector<char> specialSymbols;
  Verbosity verbosity;
  bool log_output;
  std::string log_folder;

  PcmParams pcm_params;

  // multirobot frame alignment
  MultiRobotAlignMethod multirobot_align_method;

  // GNC variables
  enum class GncThresholdMode { COST = 0u, PROBABILITY = 1u };
  bool use_gnc_;
  GncThresholdMode gnc_threshold_mode_;
  double gnc_inlier_threshold_;
};

}  // namespace KimeraRPGO