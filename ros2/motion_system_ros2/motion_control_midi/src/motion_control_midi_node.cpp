#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <midi_msgs/msg/midi.hpp>
#include <motion_control_msgs/msg/motor_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

namespace
{

constexpr std::size_t kNumMidiChannels = 8;
constexpr int32_t kFaderValueMax = 16383;
constexpr int32_t kDialValueMax = 127;
constexpr int32_t kJogUnitMax = 5000;
constexpr uint16_t kCwNewSetPointZeroerr = 0x103F;
constexpr uint16_t kCwNewSetPointMinas = 0x003F;
constexpr uint16_t kCwSocketcanSetPoint = 0x0001;
constexpr uint16_t kCwDynamixelEffortEnable = 1;

constexpr double kFaderEpsilon = 0.5;
constexpr auto kPublishPeriod = std::chrono::milliseconds(5);
constexpr double kMaxSmoothingTimeMs = 3000.0;
constexpr double kSmoothingCurvePower = 2.0;
const char * kFilesDirEnv = std::getenv("MOTION_SYSTEM_FILES_DIR");
const char * kHomeDir = std::getenv("HOME");
const std::filesystem::path kDefaultFilesDir =
  (kFilesDirEnv != nullptr && kFilesDirEnv[0] != '\0') ?
  std::filesystem::path(kFilesDirEnv) :
  ((kHomeDir != nullptr && kHomeDir[0] != '\0') ?
  std::filesystem::path(kHomeDir) / "colcon_ws" / "files" :
  std::filesystem::path("/tmp/colcon_ws/files"));

std::string to_lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

template<typename T>
T required_as(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::runtime_error("Missing '" + std::string(key) + "' in " + context);
  }
  return node[key].as<T>();
}

template<typename Vector>
bool bool_at(const Vector & values, std::size_t index)
{
  return index < values.size() && static_cast<bool>(values[index]);
}

template<typename Vector>
int32_t int_at(
  const Vector & values,
  std::size_t index,
  int32_t fallback = 0)
{
  if (index >= values.size()) {
    return fallback;
  }
  return values[index];
}

const std::filesystem::path kDefaultRobotConfigPath =
  kDefaultFilesDir / "robot_manager" / "active_robot_manager.yaml";

}  // namespace

class MotionControlMidiNode : public rclcpp::Node
{
public:
  using MidiMsg = midi_msgs::msg::Midi;
  using MotorStatus = motion_control_msgs::msg::MotorStatus;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;

  explicit MotionControlMidiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("motion_control_midi_node", options)
  {
    publish_period_ = kPublishPeriod;
    max_smoothing_time_ms_ = kMaxSmoothingTimeMs;
    smoothing_curve_power_ = kSmoothingCurvePower;
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>("robot_config_file", kDefaultRobotConfigPath.string());
    declare_parameter<bool>("jog_mode", false);
    declare_parameter<bool>("record_mode", false);
    declare_parameter<double>("home_position_tolerance", 0.5);

    std::string config_file;
    std::string robot_config_file;
    get_parameter("config_file", config_file);
    get_parameter("robot_config_file", robot_config_file);
    get_parameter("jog_mode", jog_mode_);
    get_parameter("record_mode", record_mode_);
    get_parameter("home_position_tolerance", home_position_tolerance_);
    home_position_tolerance_ = std::max(0.0, home_position_tolerance_);
    if (jog_mode_ && record_mode_) {
      RCLCPP_FATAL(get_logger(), "jog_mode and record_mode cannot both be true.");
      throw std::runtime_error("jog_mode and record_mode cannot both be true.");
    }
    if (config_file.empty()) {
      throw std::runtime_error("config_file parameter must be set.");
    }
    if (!jog_mode_ && robot_config_file.empty()) {
      throw std::runtime_error("robot_config_file parameter must be set.");
    }

    motor_config_path_ = std::filesystem::absolute(config_file).lexically_normal();
    motor_infos_ = load_motor_infos(config_file);
    if (motor_infos_.empty()) {
      throw std::runtime_error("No motor controllers were found in " + config_file);
    }
    motor_states_.resize(motor_infos_.size());
    selected_controllers_.fill(-1);
    latest_positions_.resize(motor_infos_.size());
    has_latest_position_.resize(motor_infos_.size(), false);
    latest_encoders_.resize(motor_infos_.size());
    has_latest_encoder_.resize(motor_infos_.size(), false);
    if (!jog_mode_) {
      load_robot_config(robot_config_file);
    }

    motor_command_pub_ = create_publisher<MotorStatus>(
      "motion_control/motor_command", rclcpp::QoS(1).best_effort());
    request_pub_ = create_publisher<Int8MultiArray>(
      "motion_control/request", rclcpp::QoS(1).best_effort());
    btn2_reset_pub_ = create_publisher<Int8MultiArray>(
      "/xtouch/btn2_reset", rclcpp::QoS(1).best_effort());
    btn1_set_pub_ = create_publisher<Int8MultiArray>(
      "/xtouch/btn1_set", rclcpp::QoS(1).reliable());
    btn1_reset_pub_ = create_publisher<Int8MultiArray>(
      "/xtouch/btn1_reset", rclcpp::QoS(1).reliable());

    motor_status_sub_ = create_subscription<MotorStatus>(
      "motion_control/motor_status", rclcpp::QoS(1).best_effort(),
      std::bind(&MotionControlMidiNode::motor_status_callback, this, std::placeholders::_1));

    midi_sub_ = create_subscription<MidiMsg>(
      "/xtouch/midi", rclcpp::QoS(1).best_effort(),
      std::bind(&MotionControlMidiNode::midi_callback, this, std::placeholders::_1));

    command_tick_ = create_wall_timer(
      publish_period_, std::bind(&MotionControlMidiNode::publish_motor_command, this));
  }

private:
  struct DriverInfo
  {
    uint8_t id{};
    double lower{};
    double upper{};
    double speed{};
    double rated_effort{};
    double pulse_per_revolution{};
    double gear_ratio{1.0};
    int32_t zero_offset{};
    std::string type;
  };

  struct MotorInfo
  {
    uint8_t controller_index{};
    int8_t profile_mode{};
    DriverInfo driver;
  };

  struct MotorCommandState
  {
    bool active{};
    bool initialized{};
    bool use_home_position{};
    bool home_trajectory_initialized{};
    std::size_t source_channel{};
    int32_t target_fader{};
    double current_fader{};
    double home_position{};
    double home_start_position{};
    double home_duration{};
    std::chrono::steady_clock::time_point home_start_time{};
    int32_t dial0{};
  };

  struct PendingTarget
  {
    std::size_t midi_channel{};
    std::size_t motor_index{};
    bool use_home_position{};
    int32_t fader{};
    double home_position{};
    double home_duration{};
    int32_t dial0{};
  };

  struct JogChannelState
  {
    bool dial_initialized{};
    int32_t selected_controller{-1};
    int32_t last_dial{};
    int64_t target_encoder{};
  };

  void midi_callback(const MidiMsg::SharedPtr msg)
  {
    publish_request_on_btn0_change(*msg);
    if (jog_mode_) {
      handle_jog_midi(*msg);
      return;
    }
    update_motion_recording(*msg);

    std::vector<PendingTarget> targets;
    targets.reserve(kNumMidiChannels);
    std::array<bool, kNumMidiChannels> valid_home_requests{};

    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      const bool use_home_position =
        !bool_at(msg->btn0, ch) && bool_at(msg->btn2, ch);
      if (!use_home_position && !bool_at(msg->btn3, ch)) {
        continue;
      }

      const int32_t selected_controller = int_at(
        msg->dial1, ch, static_cast<int32_t>(ch));
      if (selected_controller < 0) {
        continue;
      }
      const std::size_t motor_index = static_cast<std::size_t>(selected_controller);
      if (motor_index >= motor_infos_.size()) {
        continue;
      }

      PendingTarget target;
      target.midi_channel = ch;
      target.motor_index = motor_index;
      target.use_home_position = use_home_position;
      if (target.use_home_position) {
        const uint8_t controller_index = motor_infos_[motor_index].controller_index;
        const auto home = home_positions_.find(controller_index);
        if (home == home_positions_.end()) {
          continue;
        }
        target.home_position = home->second;
        target.home_duration = home_durations_.at(controller_index);
        valid_home_requests[ch] = true;
      } else {
        target.fader = std::clamp(
          int_at(msg->channel, ch), int32_t{0}, kFaderValueMax);
        target.dial0 = std::clamp(
          int_at(msg->dial0, ch), int32_t{0}, kDialValueMax);
      }
      targets.push_back(target);
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    for (auto & state : motor_states_) {
      state.active = false;
    }
    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      if (!valid_home_requests[ch]) {
        home_requested_[ch] = false;
        home_completed_[ch] = false;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    for (const auto & target : targets) {
      bool new_home_request = false;
      if (target.use_home_position) {
        const std::size_t ch = target.midi_channel;
        if (!home_requested_[ch] || home_motor_indices_[ch] != target.motor_index) {
          home_completed_[ch] = false;
          home_motor_indices_[ch] = target.motor_index;
          new_home_request = true;
        }
        home_requested_[ch] = true;
        if (home_completed_[ch]) {
          continue;
        }
      }
      MotorCommandState & state = motor_states_[target.motor_index];
      if (target.use_home_position) {
        if (new_home_request || !state.home_trajectory_initialized) {
          if (has_latest_position_[target.motor_index]) {
            state.home_start_position = latest_positions_[target.motor_index];
            state.home_start_time = now;
            state.home_trajectory_initialized = true;
          } else {
            state.home_trajectory_initialized = false;
          }
        }
      } else {
        if (!state.initialized || state.use_home_position) {
          state.current_fader = target.fader;
          state.initialized = true;
        }
        state.home_trajectory_initialized = false;
      }
      state.active = true;
      state.use_home_position = target.use_home_position;
      state.source_channel = target.midi_channel;
      state.target_fader = target.fader;
      state.home_position = target.home_position;
      state.home_duration = target.home_duration;
      state.dial0 = target.dial0;
    }
  }

  void handle_jog_midi(const MidiMsg & midi)
  {
    MotorStatus command = make_empty_motor_status();
    bool has_command = false;
    Int8MultiArray reset_btn1;

    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      const int32_t selected = int_at(midi.dial1, ch, -1);
      const bool valid_motor = selected >= 0 &&
        static_cast<std::size_t>(selected) < motor_infos_.size();
      const bool btn0 = bool_at(midi.btn0, ch);
      const bool btn3 = bool_at(midi.btn3, ch);
      const bool btn1 = bool_at(midi.btn1, ch);

      if (btn1 && !btn1_active_[ch]) {
        if (valid_motor && !btn0 && !btn3) {
          const std::size_t motor_index = static_cast<std::size_t>(selected);
          const double target_angle = scale_fader(
            std::clamp(int_at(midi.channel, ch), int32_t{0}, kFaderValueMax),
            motor_infos_[motor_index].driver.lower,
            motor_infos_[motor_index].driver.upper);
          update_zero_offset(motor_index, target_angle);
          reset_btn1.data.push_back(static_cast<int8_t>(ch));
        }
      }
      btn1_active_[ch] = btn1;

      JogChannelState & state = jog_channels_[ch];
      if (!valid_motor || btn0 || !btn3) {
        state.dial_initialized = false;
        state.selected_controller = selected;
        continue;
      }

      const std::size_t motor_index = static_cast<std::size_t>(selected);
      if (motor_infos_[motor_index].profile_mode != 0) {
        state.dial_initialized = false;
        continue;
      }

      const int32_t dial = int_at(midi.dial0, ch);
      if (!state.dial_initialized || state.selected_controller != selected) {
        state.selected_controller = selected;
        state.last_dial = dial;
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (!has_latest_encoder_[motor_index]) {
          state.dial_initialized = false;
          continue;
        }
        const auto [encoder_lower, encoder_upper] = encoder_limits(
          motor_infos_[motor_index].driver);
        state.target_encoder = std::clamp<int64_t>(
          latest_encoders_[motor_index], encoder_lower, encoder_upper);
        state.dial_initialized = true;
        continue;
      }

      const int64_t dial_delta = static_cast<int64_t>(dial) - state.last_dial;
      state.last_dial = dial;
      if (dial_delta == 0) {
        continue;
      }

      const int32_t jog_unit = static_cast<int32_t>(std::lround(
          static_cast<double>(std::clamp(
            int_at(midi.channel, ch), int32_t{0}, kFaderValueMax)) /
          static_cast<double>(kFaderValueMax) * kJogUnitMax));
      const auto [encoder_lower, encoder_upper] = encoder_limits(
        motor_infos_[motor_index].driver);
      const int64_t target = std::clamp<int64_t>(
        state.target_encoder + dial_delta * jog_unit,
        encoder_lower, encoder_upper);
      state.target_encoder = target;
      has_command = fill_encoder_command_target(
        command, motor_index, static_cast<int32_t>(target)) || has_command;
    }

    if (has_command) {
      motor_command_pub_->publish(command);
    }
    if (!reset_btn1.data.empty()) {
      btn1_reset_pub_->publish(reset_btn1);
    }
  }

  void motor_status_callback(const MotorStatus::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(recording_mutex_);
      if (motion_recording_active_) {
        append_motion_record_sample(*msg);
      }
    }

    Int8MultiArray reset_channels;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (std::size_t motor_index = 0; motor_index < motor_infos_.size(); ++motor_index) {
        double position = 0.0;
        if (position_for_controller(
            *msg, motor_infos_[motor_index].controller_index, position))
        {
          latest_positions_[motor_index] = position;
          has_latest_position_[motor_index] = true;
        }
        int32_t encoder = 0;
        if (encoder_for_controller(
            *msg, motor_infos_[motor_index].controller_index, encoder))
        {
          latest_encoders_[motor_index] = encoder;
          has_latest_encoder_[motor_index] = true;
        }
      }

      for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
        if (!home_requested_[ch] || home_completed_[ch]) {
          continue;
        }
        const std::size_t motor_index = home_motor_indices_[ch];
        if (motor_index >= motor_infos_.size()) {
          continue;
        }
        const uint8_t controller_index = motor_infos_[motor_index].controller_index;
        const auto home = home_positions_.find(controller_index);
        double current_position = 0.0;
        if (home == home_positions_.end() ||
          !position_for_controller(*msg, controller_index, current_position) ||
          std::abs(current_position - home->second) > home_position_tolerance_)
        {
          continue;
        }

        home_completed_[ch] = true;
        MotorCommandState & state = motor_states_[motor_index];
        if (state.use_home_position && state.source_channel == ch) {
          state.active = false;
        }
        reset_channels.data.push_back(static_cast<int8_t>(ch));
      }
    }

    if (!reset_channels.data.empty()) {
      btn2_reset_pub_->publish(reset_channels);
    }
  }

  void publish_request_on_btn0_change(const MidiMsg & msg)
  {
    Int8MultiArray request;
    request.data.assign(motor_infos_.size(), 2);

    bool should_publish = false;
    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      const bool active = bool_at(msg.btn0, ch);
      const int32_t selected_controller = int_at(
        msg.dial1, ch, static_cast<int32_t>(ch));
      const int32_t previous_controller = selected_controllers_[ch];

      const auto set_request = [&](int32_t selected, int8_t value) {
          if (selected < 0 || static_cast<std::size_t>(selected) >= motor_infos_.size()) {
            return;
          }
          const std::size_t controller_index =
            motor_infos_[static_cast<std::size_t>(selected)].controller_index;
          if (controller_index >= request.data.size()) {
            request.data.resize(controller_index + 1, 2);
          }
          request.data[controller_index] = value;
          should_publish = true;
        };

      if (active != btn0_active_[ch]) {
        set_request(selected_controller, active ? 0 : 1);
      } else if (active && selected_controller != previous_controller) {
        set_request(previous_controller, 1);
        set_request(selected_controller, 0);
      }

      btn0_active_[ch] = active;
      selected_controllers_[ch] = selected_controller;
    }

    if (should_publish) {
      request_pub_->publish(request);
    }
  }

  void update_motion_recording(const MidiMsg & msg)
  {
    bool record_mode = false;
    get_parameter("record_mode", record_mode);
    if (!record_mode) {
      stop_motion_recording(false);
      recording_session_started_ = false;
      recording_session_finished_ = false;
      recording_channels_initialized_ = false;
      recording_channels_.fill(false);
      return;
    }
    if (recording_session_finished_) {
      return;
    }

    if (!recording_channels_initialized_) {
      Int8MultiArray selected_channels;
      for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
        const int32_t selected_controller = int_at(msg.dial1, ch, -1);
        if (selected_controller < 0 ||
          static_cast<std::size_t>(selected_controller) >= motor_infos_.size())
        {
          continue;
        }
        recording_channels_[ch] = true;
        selected_channels.data.push_back(static_cast<int8_t>(ch));
      }
      if (selected_channels.data.empty()) {
        return;
      }
      recording_channels_initialized_ = true;
      btn1_set_pub_->publish(selected_channels);
      return;
    }

    bool all_btn1_active = true;
    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      if (recording_channels_[ch] && !bool_at(msg.btn1, ch)) {
        all_btn1_active = false;
        break;
      }
    }

    if (!recording_session_started_) {
      if (!all_btn1_active) {
        return;
      }
      bool selected_axis_is_manually_active = false;
      for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
        if (recording_channels_[ch] && bool_at(msg.btn3, ch) && bool_at(msg.touch, ch)) {
          selected_axis_is_manually_active = true;
          break;
        }
      }
      if (!selected_axis_is_manually_active) {
        return;
      }
      start_motion_recording();
      recording_session_started_ = true;
      return;
    }

    if (!all_btn1_active) {
      stop_motion_recording(true);
      recording_session_finished_ = true;
      Int8MultiArray selected_channels;
      for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
        if (recording_channels_[ch]) {
          selected_channels.data.push_back(static_cast<int8_t>(ch));
        }
      }
      btn1_reset_pub_->publish(selected_channels);
    }
  }

  void publish_motor_command()
  {
    if (jog_mode_) {
      return;
    }
    MotorStatus msg = make_empty_motor_status();
    bool has_target = false;

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (std::size_t motor_index = 0; motor_index < motor_states_.size(); ++motor_index) {
        MotorCommandState & state = motor_states_[motor_index];
        if (!state.active) {
          continue;
        }
        if (state.use_home_position) {
          if (!state.home_trajectory_initialized) {
            continue;
          }
          const double elapsed_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - state.home_start_time).count();
          const double progress = state.home_duration > 0.0 ?
            std::clamp(elapsed_sec / state.home_duration, 0.0, 1.0) : 1.0;
          const double progress2 = progress * progress;
          const double progress3 = progress2 * progress;
          const double smoothstep = progress3 *
            (10.0 + progress * (-15.0 + 6.0 * progress));
          const double position = state.home_start_position +
            (state.home_position - state.home_start_position) * smoothstep;
          has_target = fill_position_command_target(msg, motor_index, position) || has_target;
          continue;
        }

        const double alpha = smoothing_alpha(state.dial0);
        const double error =
          static_cast<double>(state.target_fader) - state.current_fader;
        if (std::abs(error) <= kFaderEpsilon || alpha >= 1.0) {
          state.current_fader = state.target_fader;
        } else {
          state.current_fader += error * alpha;
        }

        const int32_t fader_value = std::clamp<int32_t>(
          static_cast<int32_t>(std::lround(state.current_fader)),
          0, kFaderValueMax);
        has_target =
          fill_motor_command_target(msg, motor_index, fader_value) || has_target;
      }
    }

    if (has_target) {
      motor_command_pub_->publish(msg);
    }
  }

  double smoothing_alpha(int32_t dial) const
  {
    if (dial <= 0 || max_smoothing_time_ms_ <= 0.0) {
      return 1.0;
    }

    const double normalized =
      static_cast<double>(std::clamp(dial, int32_t{0}, kDialValueMax)) /
      static_cast<double>(kDialValueMax);
    const double curved = std::pow(normalized, smoothing_curve_power_);
    const double tau_ms = curved * max_smoothing_time_ms_;
    if (tau_ms <= 0.0) {
      return 1.0;
    }

    const double dt_ms = static_cast<double>(publish_period_.count());
    return std::clamp(1.0 - std::exp(-dt_ms / tau_ms), 0.001, 1.0);
  }

  std::vector<MotorInfo> load_motor_infos(const std::string & config_file) const
  {
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) {
      throw std::runtime_error("Failed to load motor config: " + config_file);
    }

    const YAML::Node drivers_node = root["drivers"];
    if (!drivers_node || !drivers_node.IsSequence()) {
      throw std::runtime_error("Invalid or missing 'drivers' in " + config_file);
    }

    std::unordered_map<int, DriverInfo> drivers;
    for (const auto & driver_node : drivers_node) {
      const int id = required_as<int>(driver_node, "id", "drivers[]");
      DriverInfo info;
      info.id = static_cast<uint8_t>(id);
      info.lower = required_as<double>(driver_node, "lower", "drivers[]");
      info.upper = required_as<double>(driver_node, "upper", "drivers[]");
      info.speed = required_as<double>(driver_node, "speed", "drivers[]");
      info.rated_effort =
        required_as<double>(driver_node, "rated_effort", "drivers[]");
      info.pulse_per_revolution = driver_node["pulse_per_revolution"] ?
        driver_node["pulse_per_revolution"].as<double>() : 0.0;
      info.gear_ratio = driver_node["gear_ratio"] ?
        driver_node["gear_ratio"].as<double>() : 1.0;
      info.zero_offset = driver_node["zero_offset"] ?
        driver_node["zero_offset"].as<int32_t>() : 0;
      info.type = to_lower(required_as<std::string>(driver_node, "type", "drivers[]"));
      drivers.emplace(id, info);
    }

    const YAML::Node masters_node = root["masters"];
    if (!masters_node || !masters_node.IsSequence()) {
      throw std::runtime_error("Invalid or missing 'masters' in " + config_file);
    }

    std::map<int, MotorInfo> motors_by_index;
    for (const auto & master_node : masters_node) {
      const YAML::Node slaves_node = master_node["slaves"];
      if (!slaves_node || !slaves_node.IsSequence()) {
        throw std::runtime_error("Invalid or missing 'slaves' in masters[]");
      }

      for (const auto & slave_node : slaves_node) {
        const int controller_index =
          required_as<int>(slave_node, "controller_index", "slaves[]");
        const int driver_id = required_as<int>(slave_node, "driver_id", "slaves[]");
        const int profile_mode =
          required_as<int>(slave_node, "profile_mode", "slaves[]");

        if (controller_index < 0 || controller_index > 255) {
          throw std::runtime_error("controller_index is out of uint8 range.");
        }

        const auto driver_iter = drivers.find(driver_id);
        if (driver_iter == drivers.end()) {
          throw std::runtime_error(
                  "No driver entry for driver_id " + std::to_string(driver_id));
        }

        MotorInfo info;
        info.controller_index = static_cast<uint8_t>(controller_index);
        info.profile_mode = static_cast<int8_t>(profile_mode);
        info.driver = driver_iter->second;

        const auto insert_result = motors_by_index.emplace(controller_index, info);
        if (!insert_result.second) {
          throw std::runtime_error(
                  "Duplicate controller_index " + std::to_string(controller_index));
        }
      }
    }

    std::vector<MotorInfo> result;
    result.reserve(motors_by_index.size());
    int expected_index = 0;
    for (const auto & [controller_index, info] : motors_by_index) {
      if (controller_index != expected_index) {
        throw std::runtime_error(
                "controller_index values must be dense from 0 for motor_manager.");
      }
      result.push_back(info);
      ++expected_index;
    }

    return result;
  }

  std::filesystem::path resolve_resource_path(
    const std::string & resource_path,
    const std::filesystem::path & config_directory) const
  {
    constexpr const char * package_prefix = "package://";
    if (resource_path.rfind(package_prefix, 0) == 0) {
      const std::string package_path = resource_path.substr(
        std::char_traits<char>::length(
          package_prefix));
      const std::size_t separator = package_path.find('/');
      if (separator == std::string::npos || separator == 0 ||
        separator + 1 >= package_path.size())
      {
        throw std::runtime_error("Invalid package resource path: " + resource_path);
      }
      const std::string package_name = package_path.substr(0, separator);
      return std::filesystem::path(
        ament_index_cpp::get_package_share_directory(package_name)) /
             package_path.substr(separator + 1);
    }

    const std::filesystem::path path(resource_path);
    return path.is_absolute() ? path : config_directory / path;
  }

  void load_robot_config(const std::string & config_file)
  {
    robot_config_path_ = std::filesystem::absolute(config_file).lexically_normal();
    const YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node robots = root["robot"];
    if (!robots || !robots.IsSequence() || robots.size() == 0) {
      throw std::runtime_error("Invalid or missing 'robot' in " + config_file);
    }

    const std::filesystem::path config_directory =
      std::filesystem::absolute(config_file).parent_path();
    std::filesystem::path configured_motion_path;
    for (const auto & robot : robots) {
      const auto controller_indices = required_as<std::vector<int>>(
        robot, "controller_indices", "robot[]");
      const auto home_positions = required_as<std::vector<double>>(
        robot, "home_positions", "robot[]");
      const double home_duration = required_as<double>(robot, "home_duration", "robot[]");
      if (controller_indices.size() != home_positions.size()) {
        throw std::runtime_error(
                "controller_indices and home_positions must have the same size in robot[].");
      }
      if (home_duration < 0.0) {
        throw std::runtime_error("home_duration must not be negative in robot[].");
      }

      for (std::size_t i = 0; i < controller_indices.size(); ++i) {
        const int controller_index = controller_indices[i];
        if (controller_index < 0 || controller_index > 255) {
          throw std::runtime_error("Robot controller_index is out of uint8 range.");
        }
        const auto inserted = home_positions_.emplace(
          static_cast<uint8_t>(controller_index), home_positions[i]);
        if (!inserted.second) {
          throw std::runtime_error(
                  "Duplicate robot controller_index " + std::to_string(controller_index));
        }
        home_durations_.emplace(static_cast<uint8_t>(controller_index), home_duration);
      }

      const std::string robot_name = required_as<std::string>(robot, "name", "robot[]");
      std::filesystem::path motion_path = resolve_resource_path(
        required_as<std::string>(robot, "motion_data_file_path", "robot[]"),
        config_directory);
      if (motion_path.extension() != ".csv") {
        motion_path /= robot_name + ".csv";
      }
      motion_path = motion_path.lexically_normal();
      if (!configured_motion_path.empty() && configured_motion_path != motion_path) {
        throw std::runtime_error(
                "motion_control_midi supports one motion_data_file_path per robot config.");
      }
      configured_motion_path = motion_path;
    }

    if (configured_motion_path.empty()) {
      throw std::runtime_error("No motion_data_file_path was found in " + config_file);
    }
    motion_record_path_ = configured_motion_path;
    if (record_mode_) {
      RCLCPP_INFO(
        get_logger(), "Loaded %zu home positions; motion data path is '%s'.",
        home_positions_.size(), motion_record_path_.string().c_str());
    }
  }

  MotorStatus make_empty_motor_status() const
  {
    MotorStatus msg;
    const std::size_t n = motor_infos_.size();

    msg.number_of_target_interfaces.assign(n, 0);
    msg.target_interface_id.resize(n);
    msg.controller_index.resize(n);
    msg.controlword.assign(n, 0);
    msg.statusword.assign(n, 0);
    msg.errorcode.assign(n, 0);
    msg.encoder.assign(n, 0);
    msg.position.assign(n, 0.0);
    msg.velocity.assign(n, 0.0);
    msg.effort.assign(n, 0.0);

    for (std::size_t i = 0; i < n; ++i) {
      msg.controller_index[i] = motor_infos_[i].controller_index;
    }

    return msg;
  }

  bool fill_motor_command_target(
    MotorStatus & msg, std::size_t motor_index, int32_t fader_value)
  {
    if (motor_index >= motor_infos_.size()) {
      return false;
    }

    const MotorInfo & motor = motor_infos_[motor_index];
    const std::size_t idx = motor.controller_index;

    if (idx >= msg.controller_index.size()) {
      return false;
    }

    switch (motor.profile_mode) {
      case 0: {
          return fill_position_command_target(
            msg, motor_index,
            scale_fader(fader_value, motor.driver.lower, motor.driver.upper));
        }
      case 1: {
          msg.number_of_target_interfaces[idx] = 1;
          msg.target_interface_id[idx].data = std::vector<int8_t>{2};
          const double degrees_per_second = scale_fader(
            fader_value, -motor.driver.speed, motor.driver.speed);
          msg.velocity[idx] = degrees_per_second;
          break;
        }
      case 2: {
          msg.number_of_target_interfaces[idx] = 1;
          msg.target_interface_id[idx].data = std::vector<int8_t>{3};
          msg.effort[idx] = scale_fader(
            fader_value, -motor.driver.rated_effort, motor.driver.rated_effort);
          break;
        }
      default:
        return false;
    }

    return true;
  }

  bool fill_position_command_target(
    MotorStatus & msg, std::size_t motor_index, double position)
  {
    if (motor_index >= motor_infos_.size()) {
      return false;
    }
    const MotorInfo & motor = motor_infos_[motor_index];
    const std::size_t idx = motor.controller_index;
    if (motor.profile_mode != 0 || idx >= msg.controller_index.size()) {
      return false;
    }

    if (to_lower(motor.driver.type) == "dynamixel") {
      msg.number_of_target_interfaces[idx] = 1;
      msg.target_interface_id[idx].data = std::vector<int8_t>{1};
    } else {
      msg.number_of_target_interfaces[idx] = 2;
      msg.target_interface_id[idx].data = std::vector<int8_t>{0, 1};
      msg.controlword[idx] = controlword_for_driver(motor.driver.type);
    }
    msg.position[idx] = std::clamp(
      position,
      std::min(motor.driver.lower, motor.driver.upper),
      std::max(motor.driver.lower, motor.driver.upper));
    return true;
  }

  bool fill_encoder_command_target(
    MotorStatus & msg, std::size_t motor_index, int32_t encoder)
  {
    if (motor_index >= motor_infos_.size()) {
      return false;
    }
    const MotorInfo & motor = motor_infos_[motor_index];
    const std::size_t idx = motor.controller_index;
    if (motor.profile_mode != 0 || idx >= msg.controller_index.size()) {
      return false;
    }

    if (to_lower(motor.driver.type) == "dynamixel") {
      msg.number_of_target_interfaces[idx] = 1;
      msg.target_interface_id[idx].data = std::vector<int8_t>{1};
    } else {
      msg.number_of_target_interfaces[idx] = 2;
      msg.target_interface_id[idx].data = std::vector<int8_t>{0, 1};
      msg.controlword[idx] = controlword_for_driver(motor.driver.type);
    }
    msg.encoder[idx] = encoder;
    return true;
  }

  double scale_fader(int32_t fader_value, double lower, double upper) const
  {
    const double normalized =
      static_cast<double>(std::clamp<int32_t>(fader_value, 0, kFaderValueMax)) /
      static_cast<double>(kFaderValueMax);
    return lower + (upper - lower) * normalized;
  }

  std::pair<int32_t, int32_t> encoder_limits(const DriverInfo & driver) const
  {
    if (driver.pulse_per_revolution <= 0.0 || driver.gear_ratio == 0.0) {
      return {
        std::numeric_limits<int32_t>::min(),
        std::numeric_limits<int32_t>::max()};
    }
    const auto encoder_for_angle = [&driver](double angle) {
        const double encoder = static_cast<double>(driver.zero_offset) +
          angle * driver.gear_ratio / 360.0 * driver.pulse_per_revolution;
        return static_cast<int32_t>(std::clamp<double>(
                 std::round(encoder),
                 std::numeric_limits<int32_t>::min(),
                 std::numeric_limits<int32_t>::max()));
      };
    const int32_t first = encoder_for_angle(driver.lower);
    const int32_t second = encoder_for_angle(driver.upper);
    return {std::min(first, second), std::max(first, second)};
  }

  uint16_t controlword_for_driver(const std::string & driver_type) const
  {
    const std::string type = to_lower(driver_type);
    if (type == "zeroerr") {
      return kCwNewSetPointZeroerr;
    }
    if (type == "minas") {
      return kCwNewSetPointMinas;
    }
    if (type == "cubemars") {
      return kCwSocketcanSetPoint;
    }
    if (type == "dynamixel") {
      return kCwDynamixelEffortEnable;
    }

    return kCwNewSetPointZeroerr;
  }

  bool write_motion_csv(
    const std::filesystem::path & path,
    const std::vector<std::vector<double>> & rows) const
  {
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
      return false;
    }

    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (std::size_t row = 0; row < rows.size(); ++row) {
      for (std::size_t col = 0; col < rows[row].size(); ++col) {
        if (col > 0) {
          output << ", ";
        }
        output << rows[row][col];
      }
      if (row + 1 < rows.size()) {
        output << '\n';
      }
    }

    return static_cast<bool>(output);
  }

  bool update_robot_move_duration(double duration) const
  {
    if (robot_config_path_.empty()) {
      return false;
    }

    YAML::Node root = YAML::LoadFile(robot_config_path_.string());
    YAML::Node robots = root["robot"];
    if (!robots || !robots.IsSequence() || robots.size() == 0) {
      return false;
    }

    const double rounded_duration = std::round(duration * 10.0) / 10.0;
    for (std::size_t i = 0; i < robots.size(); ++i) {
      robots[i]["move_duration"] = rounded_duration;
    }

    YAML::Emitter emitter;
    emitter << root;
    if (!emitter.good()) {
      return false;
    }

    std::filesystem::path temporary_path = robot_config_path_;
    temporary_path += ".motion_control_midi.tmp";
    {
      std::ofstream output(temporary_path, std::ios::trunc);
      if (!output) {
        return false;
      }
      output << emitter.c_str() << '\n';
      if (!output) {
        return false;
      }
    }

    std::error_code ec;
    std::filesystem::rename(temporary_path, robot_config_path_, ec);
    if (ec) {
      std::filesystem::remove(temporary_path);
      return false;
    }

    return true;
  }

  bool position_for_controller(
    const MotorStatus & status,
    uint8_t controller_index,
    double & position) const
  {
    for (std::size_t i = 0; i < status.controller_index.size(); ++i) {
      if (status.controller_index[i] != controller_index) {
        continue;
      }
      if (i >= status.position.size()) {
        return false;
      }
      position = status.position[i];
      return true;
    }

    return false;
  }

  bool encoder_for_controller(
    const MotorStatus & status,
    uint8_t controller_index,
    int32_t & encoder) const
  {
    for (std::size_t i = 0; i < status.controller_index.size(); ++i) {
      if (status.controller_index[i] != controller_index) {
        continue;
      }
      if (i >= status.encoder.size()) {
        return false;
      }
      encoder = status.encoder[i];
      return true;
    }
    return false;
  }

  bool update_zero_offset(std::size_t motor_index, double target_angle)
  {
    if (motor_index >= motor_infos_.size()) {
      return false;
    }
    DriverInfo & driver = motor_infos_[motor_index].driver;
    if (driver.pulse_per_revolution <= 0.0 || driver.gear_ratio == 0.0) {
      RCLCPP_ERROR(
        get_logger(), "Driver %u has invalid pulse_per_revolution or gear_ratio.",
        static_cast<unsigned int>(driver.id));
      return false;
    }

    int32_t current_encoder = 0;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (!has_latest_encoder_[motor_index]) {
        RCLCPP_WARN(
          get_logger(), "Cannot update driver %u zero_offset before encoder feedback.",
          static_cast<unsigned int>(driver.id));
        return false;
      }
      current_encoder = latest_encoders_[motor_index];
    }

    const double calculated = static_cast<double>(current_encoder) -
      target_angle * driver.gear_ratio / 360.0 * driver.pulse_per_revolution;
    if (!std::isfinite(calculated) ||
      calculated < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      calculated > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
      RCLCPP_ERROR(
        get_logger(), "Calculated zero_offset for driver %u is outside int32 range.",
        static_cast<unsigned int>(driver.id));
      return false;
    }
    const int32_t zero_offset = static_cast<int32_t>(std::lround(calculated));

    std::ifstream input(motor_config_path_);
    if (!input) {
      RCLCPP_ERROR(
        get_logger(), "Failed to open motor config '%s'.",
        motor_config_path_.string().c_str());
      return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
      lines.push_back(line);
    }

    const std::regex drivers_pattern(R"(^([ ]*)drivers:[ ]*(?:#.*)?$)");
    const std::regex id_pattern(R"(^([ ]*)-[ ]+id:[ ]*([^ #]+).*$)");
    const std::regex pulse_pattern(R"(^([ ]*)pulse_per_revolution:[ ]*.*$)");
    const std::regex zero_pattern(R"(^([ ]*zero_offset[ ]*:[ ]*)[^#]*([ ]*(?:#.*)?)$)");
    bool in_drivers = false;
    bool in_target = false;
    std::size_t drivers_indent = 0;
    std::size_t target_indent = 0;
    std::size_t insert_index = lines.size();
    std::string insert_indent;
    bool updated = false;

    for (std::size_t i = 0; i < lines.size(); ++i) {
      std::smatch match;
      if (!in_drivers) {
        if (std::regex_match(lines[i], match, drivers_pattern)) {
          in_drivers = true;
          drivers_indent = match[1].str().size();
        }
        continue;
      }

      const std::size_t indent = lines[i].find_first_not_of(' ');
      if (indent != std::string::npos && indent <= drivers_indent &&
        !lines[i].empty() && lines[i][0] != ' ')
      {
        break;
      }
      if (std::regex_match(lines[i], match, id_pattern)) {
        target_indent = match[1].str().size();
        try {
          in_target = std::stoll(match[2].str(), nullptr, 0) == driver.id;
        } catch (const std::exception &) {
          in_target = false;
        }
        insert_index = lines.size();
        insert_indent.clear();
        continue;
      }
      if (!in_target) {
        continue;
      }
      if (indent != std::string::npos && indent <= target_indent &&
        lines[i].find_first_not_of(' ') < lines[i].size() &&
        lines[i][lines[i].find_first_not_of(' ')] == '-')
      {
        in_target = false;
        continue;
      }
      if (std::regex_match(lines[i], match, zero_pattern)) {
        lines[i] = match[1].str() + std::to_string(zero_offset) + match[2].str();
        updated = true;
        break;
      }
      if (std::regex_match(lines[i], match, pulse_pattern)) {
        insert_index = i + 1;
        insert_indent = match[1].str();
      }
    }

    if (!updated && insert_index <= lines.size() && !insert_indent.empty()) {
      lines.insert(
        lines.begin() + static_cast<std::ptrdiff_t>(insert_index),
        insert_indent + "zero_offset: " + std::to_string(zero_offset));
      updated = true;
    }
    if (!updated) {
      RCLCPP_ERROR(
        get_logger(), "Could not locate driver %u zero_offset in '%s'.",
        static_cast<unsigned int>(driver.id), motor_config_path_.string().c_str());
      return false;
    }

    std::filesystem::path temporary_path = motor_config_path_;
    temporary_path += ".motion_control_midi.tmp";
    {
      std::ofstream output(temporary_path, std::ios::trunc);
      if (!output) {
        return false;
      }
      for (const auto & output_line : lines) {
        output << output_line << '\n';
      }
      if (!output) {
        std::filesystem::remove(temporary_path);
        return false;
      }
    }

    std::error_code ec;
    const auto permissions = std::filesystem::status(motor_config_path_, ec).permissions();
    if (!ec) {
      std::filesystem::permissions(temporary_path, permissions, ec);
      ec.clear();
    }
    std::filesystem::rename(temporary_path, motor_config_path_, ec);
    if (ec) {
      std::filesystem::remove(temporary_path);
      RCLCPP_ERROR(
        get_logger(), "Failed to replace motor config '%s': %s",
        motor_config_path_.string().c_str(), ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Updated driver %u zero_offset to %d (target angle %.3f).",
      static_cast<unsigned int>(driver.id), zero_offset, target_angle);
    driver.zero_offset = zero_offset;
    return true;
  }

  void start_motion_recording()
  {
    std::lock_guard<std::mutex> lk(recording_mutex_);
    if (motion_recording_active_) {
      return;
    }
    motion_recording_rows_.clear();
    motion_recording_start_time_ = std::chrono::steady_clock::now();
    motion_recording_active_ = true;
    RCLCPP_INFO(
      get_logger(), "Recording started: %s", motion_record_path_.string().c_str());
  }

  void stop_motion_recording(bool save)
  {
    std::vector<std::vector<double>> rows;
    {
      std::lock_guard<std::mutex> lk(recording_mutex_);
      if (!motion_recording_active_) {
        return;
      }
      motion_recording_active_ = false;
      rows.swap(motion_recording_rows_);
    }

    if (save && !rows.empty()) {
      save_motion_record(rows);
    } else if (save) {
      RCLCPP_WARN(
        get_logger(),
        "Motion recording stopped, but no position samples were captured.");
    }
  }

  void append_motion_record_sample(const MotorStatus & status)
  {
    std::size_t row_count = 0;
    for (const auto & motor : motor_infos_) {
      row_count = std::max(row_count, static_cast<std::size_t>(motor.controller_index) + 1);
    }
    if (row_count == 0) {
      return;
    }

    std::vector<double> sample(row_count, 0.0);
    std::vector<bool> has_sample(row_count, false);
    bool has_recorded_value = false;
    for (const auto & motor : motor_infos_) {
      const std::size_t row = motor.controller_index;
      if (motor.profile_mode != 0 || row >= row_count) {
        continue;
      }

      double position = 0.0;
      if (!position_for_controller(status, motor.controller_index, position)) {
        continue;
      }

      sample[row] = position;
      has_sample[row] = true;
      has_recorded_value = true;
    }

    if (!has_recorded_value) {
      return;
    }

    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - motion_recording_start_time_).count();

    motion_recording_rows_.resize(row_count + 1);
    motion_recording_rows_[0].push_back(elapsed_sec);
    for (std::size_t row = 0; row < row_count; ++row) {
      const double value =
        has_sample[row] ? sample[row] :
        (motion_recording_rows_[row + 1].empty() ? 0.0 : motion_recording_rows_[row + 1].back());
      motion_recording_rows_[row + 1].push_back(value);
    }
  }

  void save_motion_record(const std::vector<std::vector<double>> & rows) const
  {
    std::filesystem::path path;
    try {
      path = motion_record_path_;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve motion record path: %s", e.what());
      return;
    }

    std::error_code ec;
    const std::filesystem::path directory = path.parent_path();
    if (!directory.empty()) {
      std::filesystem::create_directories(directory, ec);
    }
    if (ec) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create motion record directory '%s': %s",
        directory.string().c_str(),
        ec.message().c_str());
      return;
    }

    if (!write_motion_csv(path, rows)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to write motion record '%s'.",
        path.string().c_str());
      return;
    }

    const double recorded_duration =
      (!rows.empty() && !rows.front().empty()) ? rows.front().back() : 0.0;
    if (!update_robot_move_duration(recorded_duration)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to update move_duration in '%s'.",
        robot_config_path_.string().c_str());
    }

    RCLCPP_INFO(
      get_logger(), "Recording stopped and saved: %s", path.string().c_str());
  }

  rclcpp::Subscription<MidiMsg>::SharedPtr midi_sub_;
  rclcpp::Subscription<MotorStatus>::SharedPtr motor_status_sub_;
  rclcpp::Publisher<MotorStatus>::SharedPtr motor_command_pub_;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr request_pub_;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr btn2_reset_pub_;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr btn1_set_pub_;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr btn1_reset_pub_;
  rclcpp::TimerBase::SharedPtr command_tick_;

  std::vector<MotorInfo> motor_infos_;
  std::vector<MotorCommandState> motor_states_;
  std::filesystem::path motor_config_path_;
  std::unordered_map<uint8_t, double> home_positions_;
  std::unordered_map<uint8_t, double> home_durations_;
  std::filesystem::path motion_record_path_;
  std::filesystem::path robot_config_path_;
  std::vector<double> latest_positions_;
  std::vector<bool> has_latest_position_;
  std::vector<int32_t> latest_encoders_;
  std::vector<bool> has_latest_encoder_;
  std::array<bool, kNumMidiChannels> home_requested_{};
  std::array<bool, kNumMidiChannels> home_completed_{};
  std::array<std::size_t, kNumMidiChannels> home_motor_indices_{};
  std::array<bool, kNumMidiChannels> btn0_active_{};
  std::array<int32_t, kNumMidiChannels> selected_controllers_{};
  std::array<bool, kNumMidiChannels> btn1_active_{};
  std::array<JogChannelState, kNumMidiChannels> jog_channels_{};
  std::mutex state_mutex_;
  std::mutex recording_mutex_;
  std::vector<std::vector<double>> motion_recording_rows_;
  std::chrono::steady_clock::time_point motion_recording_start_time_{};
  bool motion_recording_active_{};
  bool recording_session_started_{};
  bool recording_session_finished_{};
  bool recording_channels_initialized_{};
  std::array<bool, kNumMidiChannels> recording_channels_{};
  std::chrono::milliseconds publish_period_{5};
  double max_smoothing_time_ms_{3000.0};
  double smoothing_curve_power_{2.0};
  double home_position_tolerance_{0.5};
  bool jog_mode_{false};
  bool record_mode_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionControlMidiNode>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
