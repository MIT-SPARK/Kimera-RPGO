/*
Simple logger class for prints and warnings
author: Yun Chang
*/

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

namespace KimeraRPGO {

enum log_level_t {
  WARNING,
  INFO,
};

namespace log_impl {

class formatted_log_t {
 public:
  formatted_log_t(log_level_t level, const char* msg)
      : fmt(msg), level(level) {}

  ~formatted_log_t() {
    if (level == 0) std::cout << "\033[1;33m" << fmt << "\033[0m" << std::endl;
    if (level == 1) std::cout << "\033[32m" << fmt << "\033[0m" << std::endl;
  }

  template <typename T>
  formatted_log_t& operator%(T value) {
    fmt % value;
    return *this;
  }

 protected:
  boost::format fmt;
  log_level_t level;
};

}  // namespace log_impl

// Helper function. Class formatted_log_t will not be used directly.
template <log_level_t level>
log_impl::formatted_log_t log(const char* msg) {
  return log_impl::formatted_log_t(level, msg);
}

void writeG2o(const gtsam::NonlinearFactorGraph& graph,
              const gtsam::Values& estimate,
              const std::string& filename);

}  // namespace KimeraRPGO
