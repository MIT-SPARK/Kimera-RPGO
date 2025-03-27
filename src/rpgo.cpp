#include "kimera_rpgo/rpgo.h"

#include <nlohmann/json.hpp>

#include "kimera_rpgo/solver/gnc_solver.h"
#include "kimera_rpgo/solver/gradient_solver.h"
#include "kimera_rpgo/solver/least_squares_solver.h"
#include "kimera_rpgo/utils/g2o.h"
#include "kimera_rpgo/utils/utils.h"
using json = nlohmann::json;

namespace kimera_rpgo {

std::ostream& operator<<(std::ostream& os, const RpgoConfig::SolverType& type) {
  switch (type) {
    case RpgoConfig::SolverType::LEAST_SQUARES:
      os << "Least-Squares";
      break;
    case RpgoConfig::SolverType::GRADIENT:
      os << "Gradient-Descent";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const RpgoConfig& config) {
  os << "===== RPGO Config =====\n";
  switch (config.solver_type) {
    case RpgoConfig::SolverType::LEAST_SQUARES:
      os << "solver_type: LEAST-SQUARES\n";
      os << "least_squares_option: "
         << config.solver_config.least_squares_option << "\n";
      break;
    case RpgoConfig::SolverType::GRADIENT:
      os << "solver_type: GRADIENT\n";
      os << "gradient_option: " << config.solver_config.gradient_option << "\n";
      break;
  }
  os << *config.solver_config.optimizer_params << "\n";
  os << "Use PCM: " << config.use_pcm << "\n";
  if (config.use_pcm) {
    os << config.pcm_config << "\n";
  }
  bool use_gnc = (config.solver_config.gnc_params) ? true : false;
  os << "Use GNC: " << use_gnc << "\n";
  if (use_gnc) {
    os << *config.solver_config.gnc_params << "\n";
  }
  os << "loss_type: " << config.loss_type << "\n";
  os << "loss_threshold_c: " << config.loss_threshold_c << "\n";
  return os;
}

void RpgoConfig::print() const { std::cout << *this; }

Rpgo::Rpgo(const RpgoConfig& config, bool print_config) : config_(config) {
  if (print_config) {
    config_.print();
  }

  switch (config_.solver_type) {
    case RpgoConfig::SolverType::LEAST_SQUARES:
      if (config_.solver_config.gnc_params) {
        solver_.reset(new GncSolver(config_.solver_config));
      } else {
        solver_.reset(new LeastSquaresSolver(config_.solver_config));
      }
      break;
    case RpgoConfig::SolverType::GRADIENT:
      if (config_.solver_config.gnc_params) {
        throw std::invalid_argument("Cannot run GNC with gradient descent");
      }
      solver_.reset(new GradientSolver(config_.solver_config));
      break;
    default:
      throw std::invalid_argument("Unexpected Solver type");
      break;
  }
  if (config_.use_pcm) {
    pcm_.reset(new Pcm(config_.pcm_config));
  }
}

Rpgo::~Rpgo() {}

void Rpgo::clear() {
  corrupted_odom_factors_.clear();
  initial_ = gtsam::Values();
  factors_ = gtsam::NonlinearFactorGraph();
  result_ = gtsam::Values();
  inlier_weights_.clear();

  solver_->clear();
  log_ = RpgoLog();
}

void Rpgo::loadG2o(const std::string& g2o, bool is_3d) {
  // Load g2o
  const auto& graph_values = gtsam::readG2owithLmks(
      g2o, is_3d, config_.loss_type, config_.loss_threshold_c);
  factors_ = *graph_values.first;
  initial_ = *graph_values.second;
}

void Rpgo::addFactors(const gtsam::NonlinearFactorGraph& factors) {
  factors_.add(factors);
}

void Rpgo::addValues(const gtsam::Values& values) {
  for (const auto& key_value : values) {
    if (initial_.exists(key_value.key)) {
      continue;
    }
    initial_.insert(key_value.key, key_value.value);

    // TODO(Yun): interpolate then add?
    result_.insert(key_value.key, key_value.value);
  }
}

void Rpgo::fixFirstPose(bool is_3d) {
  auto first_key = initial_.keys().front();
  if (is_3d) {
    factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
        first_key,
        initial_.at<gtsam::Pose3>(first_key),
        gtsam::noiseModel::Isotropic::Precision(6, 1.0e+4)));
  } else {
    factors_.add(gtsam::PriorFactor<gtsam::Pose2>(
        first_key,
        initial_.at<gtsam::Pose2>(first_key),
        gtsam::noiseModel::Isotropic::Precision(3, 1.0e+4)));
  }
}

void Rpgo::setKnownInliers(std::set<size_t> inliers) {
  inlier_factors_ = inliers;
}

void Rpgo::setCorruptedOdom(size_t index) {
  corrupted_odom_factors_.insert(index);
}

void Rpgo::run() {
  // Call solver to optimize and update result
  log_.num_variables = initial_.size();
  log_.num_factors = factors_.size();

  gtsam::NonlinearFactorGraph consistent_factors = factors_;

  auto start = std::chrono::system_clock::now();
  if (config_.use_pcm) {
    pcm_->processBatch(factors_, corrupted_odom_factors_);
    consistent_factors = pcm_->getInlierGraph();
    // TODO(Yun) hacky. Cleaner way to "initialize" after PCM?
    auto temp_solver = LeastSquaresSolver(config_.solver_config);
    std::vector<double> temp_weights;
    initial_ = temp_solver.solve(consistent_factors, initial_, temp_weights);
  }

  std::vector<double> solver_inlier_weights;
  solver_->setCorruptedOdom(corrupted_odom_factors_);
  solver_->setKnownInliers(inlier_factors_);
  result_ = solver_->solve(consistent_factors, initial_, solver_inlier_weights);

  // Construct proper weights (accounting for PCM)
  if (config_.use_pcm) {
    auto pcm_inliers = pcm_->getInlierIndices();
    inlier_weights_ = std::vector<double>(factors_.size(), 0);
    for (size_t i = 0; i < pcm_inliers.size(); i++) {
      auto idx = pcm_inliers[i];
      inlier_weights_[idx] = solver_inlier_weights[i];
    }
  } else {
    inlier_weights_ = solver_inlier_weights;
  }
  auto end = std::chrono::system_clock::now();
  log_.elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  if (config_.print_summary) {
    size_t num_inliers = 0;
    for (const auto& inlier : inlier_weights_) {
      if (inlier > 0.5) {
        num_inliers++;
      }
    }
    std::cout << "Optimized in " << log_.elapsed.count() << "(ms). "
              << factors_.size() << " total factors with " << num_inliers
              << " inliers\n";
  }
}

void Rpgo::writeResult(const std::string& output_g2o) const {
  // Save result to g2o
  saveG2o(factors_, result_, output_g2o);
}

void Rpgo::writeLog(const std::string& output_json) const {
  const auto& solver_log = solver_->getLog();
  json log;

  log["#variables"] = log_.num_variables;
  log["#factors"] = log_.num_factors;
  log["elapsed_ms"] = log_.elapsed.count();
  log["#solver_ariables"] = solver_log.num_variables;
  log["#solver_factors"] = solver_log.num_factors;
  log["solver_elapsed_ms"] = solver_log.elapsed.count();

  std::ofstream file(output_json);
  file << log;
}

}  // namespace kimera_rpgo
