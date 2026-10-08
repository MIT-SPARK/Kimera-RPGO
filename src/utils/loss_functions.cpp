#include "kimera_rpgo/utils/loss_functions.h"

namespace gtsam {
namespace noiseModel {
namespace mEstimator {

/* ************************************************************************* */
// Truncated L2
/* ************************************************************************* */

TruncatedL2::TruncatedL2(double k, const ReweightScheme reweight)
    : Base(reweight), k_(k) {
  if (k_ <= 0) {
    throw std::runtime_error(
        "mEstimator Huber takes only positive double in constructor.");
  }
}

double TruncatedL2::weight(double distance) const {
  const double absError = std::abs(distance);
  if (absError <= k_) {  // |x| <= k
    return 1.0;
  } else {  // |x| > k
    return 0.0;
  }
}

double TruncatedL2::loss(double distance) const {
  const double absError = std::abs(distance);
  if (absError <= k_) {  // |x| <= k
    return distance * distance / 2;
  } else {  // |x| > k
    return k_ * k_ / 2;
  }
}

void TruncatedL2::print(const std::string& s = "") const {
  std::cout << s << "truncated L2 (" << k_ << ")" << std::endl;
}

bool TruncatedL2::equals(const Base& expected, double tol) const {
  const TruncatedL2* p = dynamic_cast<const TruncatedL2*>(&expected);
  if (p == nullptr) return false;
  return std::abs(k_ - p->k_) < tol;
}

TruncatedL2::shared_ptr TruncatedL2::Create(double c,
                                            const ReweightScheme reweight) {
  return shared_ptr(new TruncatedL2(c, reweight));
}

/* ************************************************************************* */
// Max Consensus
/* ************************************************************************* */

MaxConsensus::MaxConsensus(double k, const ReweightScheme reweight)
    : Base(reweight), k_(k) {
  if (k_ <= 0) {
    throw std::runtime_error(
        "mEstimator MaxConsensus takes only positive double in constructor.");
  }
}

double MaxConsensus::weight(double distance) const {
  if (distance == k_) {
    return 1;
  }

  return 0;
}

double MaxConsensus::loss(double distance) const {
  const double abs_error = std::abs(distance);
  return (abs_error < k_) ? 0.0 : 1.0;
}

void MaxConsensus::print(const std::string& s = "") const {
  std::cout << s << ": MaxConsensus (" << k_ << ")" << std::endl;
}

bool MaxConsensus::equals(const Base& expected, double tol) const {
  const MaxConsensus* p = dynamic_cast<const MaxConsensus*>(&expected);
  if (p == nullptr) return false;
  return std::abs(k_ - p->k_) < tol;
}

MaxConsensus::shared_ptr MaxConsensus::Create(double k,
                                              const ReweightScheme reweight) {
  return shared_ptr(new MaxConsensus(k, reweight));
}

}  // namespace mEstimator
}  // namespace noiseModel
}  // namespace gtsam
