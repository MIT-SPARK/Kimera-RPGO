#include "kimera_rpgo/solver/solver.h"

#include <fstream>
#include <iostream>

namespace kimera_rpgo {
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

void SolverLog::print() const {
  std::cout << "===== Solver Log =====\n";
  std::cout << "# vars: " << num_variables << "\n";
  std::cout << "# factors: " << num_factors << "\n";
  std::cout << "Elapsed (ms): " << elapsed.count() << "\n";
}

Solver::Solver(const SolverConfig& config) : config_(config) {}

Solver::~Solver() {}

void Solver::clear() {
 corrupted_odom_indices_.clear();
 known_inliers_.clear();
}

gtsam::Values Solver::solve(const NonlinearFactorGraph& factors,
                            const Values& initial,
                            std::vector<double>& weights) {
  log_.num_variables = initial.size();
  log_.num_factors = factors.size();
  auto start = std::chrono::system_clock::now();

  // Figure out better way to handle logging iteration errors
  const auto results = optimize(factors, initial, weights);
  auto end = std::chrono::system_clock::now();
  log_.elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  return results;
}

}  // namespace kimera_rpgo
