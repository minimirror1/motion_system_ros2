#include <gtest/gtest.h>

#include <string>

#include "motion_control_bridge/status_publish_policy.hpp"

namespace
{

using motion_control_bridge::RuntimeProfile;
using motion_control_bridge::is_raspberry_pi;
using motion_control_bridge::parse_runtime_profile;
using motion_control_bridge::select_status_publish_rate_hz;

TEST(StatusPublishPolicy, ParsesSupportedProfiles)
{
  EXPECT_EQ(parse_runtime_profile("auto"), RuntimeProfile::Auto);
  EXPECT_EQ(parse_runtime_profile("desktop"), RuntimeProfile::Desktop);
  EXPECT_EQ(parse_runtime_profile("embedded"), RuntimeProfile::Embedded);
  EXPECT_THROW(parse_runtime_profile("raspberry_pi"), std::invalid_argument);
}

TEST(StatusPublishPolicy, DetectsRaspberryPiFromDeviceTreeCompatible)
{
  const char compatible_data[] = "raspberrypi,5-model-b\0brcm,bcm2712";
  const std::string compatible(
    compatible_data,
    sizeof(compatible_data) - 1);

  EXPECT_TRUE(is_raspberry_pi("", compatible, ""));
}

TEST(StatusPublishPolicy, DetectsRaspberryPiFromModelFallbacks)
{
  EXPECT_TRUE(is_raspberry_pi("Raspberry Pi 5 Model B Rev 1.1", "", ""));
  EXPECT_TRUE(is_raspberry_pi("", "", "Model : Raspberry Pi 4 Model B"));
  EXPECT_FALSE(is_raspberry_pi("", "", "model name : Intel(R) Core(TM)"));
}

TEST(StatusPublishPolicy, SelectsAutomaticRateForPlatform)
{
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Auto, 0, true),
    motion_control_bridge::kEmbeddedStatusPublishRateHz);
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Auto, 0, false),
    motion_control_bridge::kDesktopStatusPublishRateHz);
}

TEST(StatusPublishPolicy, ExplicitProfileOverridesDetectedPlatform)
{
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Desktop, 0, true),
    motion_control_bridge::kDesktopStatusPublishRateHz);
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Embedded, 0, false),
    motion_control_bridge::kEmbeddedStatusPublishRateHz);
}

TEST(StatusPublishPolicy, ExplicitRateHasHighestPriority)
{
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Auto, 250, true),
    250);
  EXPECT_EQ(
    select_status_publish_rate_hz(RuntimeProfile::Embedded, 500, false),
    500);
}

TEST(StatusPublishPolicy, RejectsInvalidRates)
{
  EXPECT_THROW(
    select_status_publish_rate_hz(RuntimeProfile::Auto, -1, true),
    std::invalid_argument);
  EXPECT_THROW(
    select_status_publish_rate_hz(
      RuntimeProfile::Auto,
      motion_control_bridge::kMaximumStatusPublishRateHz + 1,
      false),
    std::invalid_argument);
}

}  // namespace
