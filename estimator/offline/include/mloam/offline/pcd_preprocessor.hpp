#pragma once

#include <stdexcept>
#include <string>

#include "mloam/offline/types.hpp"

namespace mloam {
namespace offline {

class PcdValidationError : public std::runtime_error {
 public:
  explicit PcdValidationError(const std::string& message)
      : std::runtime_error(message) {}
};

PreparedLidarFrame preparePcd(const std::string& path,
                              const LidarConfig& config,
                              double minimum_finite_ratio);

}  // namespace offline
}  // namespace mloam
