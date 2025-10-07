#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <string>

#include "kimera_rpgo/pcm/pcm.h"
#include "kimera_rpgo/solver/solver.h"
#include "kimera_rpgo/utils/types.h"

namespace kimera_rpgo {

struct RpgoLog {
  size_t num_variables;
  size_t num_factors;

  std::chrono::milliseconds elapsed;
};

struct RpgoConfig {
  enum class SolverType {
    LEAST_SQUARES,
    GRADIENT
  } solver_type = SolverType::LEAST_SQUARES;

  bool use_pcm = false;

  SolverConfig solver_config;
  PcmConfig pcm_config;

  LossType loss_type = LossType::LS;
  double loss_threshold_c = 0.0;  // For loss_type

  bool print_summary = true;
  void print() const;
};

std::ostream& operator<<(std::ostream& os, const RpgoConfig::SolverType& type);

std::ostream& operator<<(std::ostream& os, const RpgoConfig& config);

class Rpgo {
 public:
  Rpgo(const RpgoConfig& config, bool print_config = true);
  ~Rpgo();

  void clear();

  void loadG2o(const std::string& g2o, bool is_3d);

  void addFactors(const gtsam::NonlinearFactorGraph& factors);

  void addValues(const gtsam::Values& values);

  void fixFirstPose(bool is_3d);

  void setKnownInliers(std::set<size_t> indices);

  void setCorruptedOdom(size_t index);

  void run();

  void writeResult(const std::string& output_g2o) const;

  void writeLog(const std::string& output_json) const;

  const gtsam::Values& getResult() const { return result_; }

  const gtsam::NonlinearFactorGraph& getFactors() const { return factors_; }

  const std::vector<double>& getInlierWeights() const {
    return inlier_weights_;
  }

  const SolverLog& getSolverLog() const { return solver_->getLog(); }

  const RpgoLog& getLog() const { return log_; }

  void setIterationCallback(const Solver::IterationCallback& callback) {
    solver_->setIterationCallback(callback);
  }

 private:
  std::unique_ptr<Solver> solver_;
  std::unique_ptr<Pcm> pcm_;

  std::set<size_t> inlier_factors_;

  std::set<size_t> corrupted_odom_factors_;

  gtsam::Values initial_;
  gtsam::NonlinearFactorGraph factors_;

  gtsam::Values result_;
  std::vector<double> inlier_weights_;

  RpgoConfig config_;
  RpgoLog log_;
};

}  // namespace kimera_rpgo
