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

struct RobustSolverParams {
 public:
  RobustSolverParams()
      : solver(Solver::LM),  // TODO(Luca): default should be GN (faster, but we
                             // can keep LM for subt)
        outlierRemovalMethod(OutlierRemovalMethod::PCM3D),
        specialSymbols(),
        verbosity(Verbosity::UPDATE),
        pcm_odomThreshold(10.0),
        pcm_lcThreshold(5.0),
        pcmDist_transThreshold(0.05),  // 5cm
        pcmDist_rotThreshold(0.005),   // <0.5degrees
        incremental(false),
        log_output(false),
        use_gnc_(false) {}
  /*! \brief For RobustSolver to not do outlier rejection at all
   */
  void setNoRejection(Verbosity verbos = Verbosity::UPDATE) {
    outlierRemovalMethod = OutlierRemovalMethod::NONE;
    verbosity = verbos;
  }

  /*! \brief use incremental max clique
   */
  void setIncremental() { incremental = true; }

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
    pcm_odomThreshold = odomThreshold;
    pcm_lcThreshold = lcThreshold;
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
    pcm_odomThreshold = odomThreshold;
    pcm_lcThreshold = lcThreshold;
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
    pcmDist_transThreshold = transThreshold;
    pcmDist_rotThreshold = rotThreshold;
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
    pcmDist_transThreshold = transThreshold;
    pcmDist_rotThreshold = rotThreshold;
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

  // for Pcm
  double pcm_odomThreshold;
  double pcm_lcThreshold;

  // for PcmSimple
  double pcmDist_transThreshold;
  double pcmDist_rotThreshold;

  // incremental max clique
  bool incremental;

  // GNC variables
  enum class GncThresholdMode { COST = 0u, PROBABILITY = 1u };
  bool use_gnc_;
  GncThresholdMode gnc_threshold_mode_;
  double gnc_inlier_threshold_;
};

}  // namespace KimeraRPGO