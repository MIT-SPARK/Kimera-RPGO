/*
Common types 
Author: Yun Chang
*/

#ifndef TYPE_UTILS_H
#define TYPE_UTILS_H

#include <memory>

namespace RobustPGO {

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}
#endif