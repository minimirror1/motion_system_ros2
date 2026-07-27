#ifndef MOTION_CONTROL_BRIDGE__STATUS_PUBLISH_POLICY_HPP_
#define MOTION_CONTROL_BRIDGE__STATUS_PUBLISH_POLICY_HPP_

#include <cstdint>
#include <string_view>

namespace motion_control_bridge
{

inline constexpr int kDesktopStatusPublishRateHz = 1000;
inline constexpr int kEmbeddedStatusPublishRateHz = 100;
inline constexpr int kMaximumStatusPublishRateHz = 10000;

enum class RuntimeProfile
{
  Auto,
  Desktop,
  Embedded,
};

RuntimeProfile parse_runtime_profile(std::string_view value);

std::string_view runtime_profile_name(RuntimeProfile profile);

bool is_raspberry_pi(
  std::string_view device_tree_model,
  std::string_view device_tree_compatible,
  std::string_view cpu_info);

int select_status_publish_rate_hz(
  RuntimeProfile profile,
  std::int64_t requested_rate_hz,
  bool raspberry_pi);

}  // namespace motion_control_bridge

#endif  // MOTION_CONTROL_BRIDGE__STATUS_PUBLISH_POLICY_HPP_
