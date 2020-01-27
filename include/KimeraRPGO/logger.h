/*
Symple logger class for prints and warnings
author: Yun Chang
*/

#ifndef INCLUDE_KIMERARPGO_LOGGER_H_
#define INCLUDE_KIMERARPGO_LOGGER_H_

#include <boost/format.hpp>
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;

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
    if (level == 0) cout << "\033[1;33m" << fmt << "\033[0m" << endl;
    if (level == 1) cout << "\033[32m" << fmt << "\033[0m" << endl;
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

}  // namespace KimeraRPGO

#endif  // INCLUDE_KIMERARPGO_LOGGER_H_
