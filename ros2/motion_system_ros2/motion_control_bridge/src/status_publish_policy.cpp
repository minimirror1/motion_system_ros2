#include "motion_control_bridge/status_publish_policy.hpp"

#include <stdexcept>
#include <string>

namespace motion_control_bridge
{

RuntimeProfile parse_runtime_profile(std::string_view value)
{
  if (value == "auto") {
    return RuntimeProfile::Auto;
  }
  if (value == "desktop") {
    return RuntimeProfile::Desktop;
  }
  if (value == "embedded") {
    return RuntimeProfile::Embedded;
  }

  throw std::invalid_argument(
          "runtime_profile must be one of: auto, desktop, embedded; got '" +
          std::string(value) + "'");
}

std::string_view runtime_profile_name(RuntimeProfile profile)
{
  switch (profile) {
    case RuntimeProfile::Auto:
      return "auto";
    case RuntimeProfile::Desktop:
      return "desktop";
    case RuntimeProfile::Embedded:
      return "embedded";
  }

  throw std::invalid_argument("Unknown runtime profile.");
}

bool is_raspberry_pi(
  std::string_view device_tree_model,
  std::string_view device_tree_compatible,
  std::string_view cpu_info)
{
  return device_tree_compatible.find("raspberrypi,") != std::string_view::npos ||
         device_tree_model.find("Raspberry Pi") != std::string_view::npos ||
         cpu_info.find("Raspberry Pi") != std::string_view::npos;
}

int select_status_publish_rate_hz(
  RuntimeProfile profile,
  std::int64_t requested_rate_hz,
  bool raspberry_pi)
{
  if (requested_rate_hz < 0 ||
    requested_rate_hz > kMaximumStatusPublishRateHz)
  {
    throw std::invalid_argument(
            "motor_status_publish_rate_hz must be 0 (automatic) or between 1 and " +
            std::to_string(kMaximumStatusPublishRateHz) + " Hz.");
  }

  if (requested_rate_hz > 0) {
    return static_cast<int>(requested_rate_hz);
  }

  switch (profile) {
    case RuntimeProfile::Auto:
      return raspberry_pi ?
             kEmbeddedStatusPublishRateHz :
             kDesktopStatusPublishRateHz;
    case RuntimeProfile::Desktop:
      return kDesktopStatusPublishRateHz;
    case RuntimeProfile::Embedded:
      return kEmbeddedStatusPublishRateHz;
  }

  throw std::invalid_argument("Unknown runtime profile.");
}

}  // namespace motion_control_bridge
