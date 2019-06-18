/* 
Symple logger class for prints and warnings 
author: Yun Chang
*/

#ifndef LOGGER_H
#define LOGGER_H

#include <sstream>
#include <boost/format.hpp>
#include <iostream>
using namespace std;

enum log_level_t {
  WARNING,
  INFO,
};

namespace log_impl {
class formatted_log_t {
public:
  formatted_log_t( log_level_t level, const char* msg ) : fmt(msg), level(level) {}
  ~formatted_log_t() {
    if (level == 0) cout << "\033[1;33m" << fmt << "\033[0m" << endl;
    if (level == 1) cout << "\033[32m" << fmt << "\033[0m" << endl;
  }        
  template <typename T> 
  formatted_log_t& operator %(T value) {
    fmt % value;
    return *this;
  }    
protected:
  log_level_t level;
  boost::format fmt;
};
}//namespace log_impl

// Helper function. Class formatted_log_t will not be used directly.
template <log_level_t level>
log_impl::formatted_log_t log(const char* msg) {
  return log_impl::formatted_log_t( level, msg );
}

#endif 