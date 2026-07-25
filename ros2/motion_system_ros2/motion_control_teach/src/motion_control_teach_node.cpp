#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <motion_control_msgs/msg/motor_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

constexpr int8_t kTorqueOff = 0;
constexpr int8_t kTorqueOn = 1;
constexpr int8_t kNoChange = 2;

const char * kFilesDirEnv = std::getenv("MOTION_SYSTEM_FILES_DIR");
const char * kHomeDir = std::getenv("HOME");
const std::filesystem::path kDefaultFilesDir =
  (kFilesDirEnv != nullptr && kFilesDirEnv[0] != '\0') ?
  std::filesystem::path(kFilesDirEnv) :
  ((kHomeDir != nullptr && kHomeDir[0] != '\0') ?
  std::filesystem::path(kHomeDir) / "colcon_ws" / "files" :
  std::filesystem::path("/tmp/colcon_ws/files"));

const std::filesystem::path kDefaultRobotConfigPath =
  kDefaultFilesDir / "robot_manager" / "active_robot_manager.yaml";

template<typename T>
T required_as(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::runtime_error("Missing '" + std::string(key) + "' in " + context);
  }
  return node[key].as<T>();
}

std::filesystem::path resolve_resource_path(
  const std::string & resource_path,
  const std::filesystem::path & config_directory)
{
  constexpr const char * package_prefix = "package://";
  if (resource_path.rfind(package_prefix, 0) == 0) {
    const std::string package_path = resource_path.substr(
      std::char_traits<char>::length(package_prefix));
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

}  // namespace

class MotionControlTeachNode : public rclcpp::Node
{
public:
  using MotorStatus = motion_control_msgs::msg::MotorStatus;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;
  using Trigger = std_srvs::srv::Trigger;

  explicit MotionControlTeachNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("motion_control_teach_node", options)
  {
    const std::string robot_config_file = declare_parameter<std::string>(
      "robot_config_file", kDefaultRobotConfigPath.string());
    if (robot_config_file.empty()) {
      throw std::runtime_error("robot_config_file parameter must be set.");
    }
    load_robot_config(robot_config_file);

    request_pub_ = create_publisher<Int8MultiArray>(
      "motion_control/request", rclcpp::QoS(1).best_effort());

    motor_status_sub_ = create_subscription<MotorStatus>(
      "motion_control/motor_status", rclcpp::QoS(1).best_effort(),
      std::bind(&MotionControlTeachNode::motor_status_callback, this, std::placeholders::_1));

    torque_off_srv_ = create_service<Trigger>(
      "~/torque_off",
      std::bind(
        &MotionControlTeachNode::handle_torque_off, this,
        std::placeholders::_1, std::placeholders::_2));
    torque_on_srv_ = create_service<Trigger>(
      "~/torque_on",
      std::bind(
        &MotionControlTeachNode::handle_torque_on, this,
        std::placeholders::_1, std::placeholders::_2));
    start_recording_srv_ = create_service<Trigger>(
      "~/start_recording",
      std::bind(
        &MotionControlTeachNode::handle_start_recording, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = create_service<Trigger>(
      "~/stop_recording",
      std::bind(
        &MotionControlTeachNode::handle_stop_recording, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "motion_control_teach_node ready: %zu controllers, motion data '%s'.",
      controller_indices_.size(), motion_record_path_.string().c_str());
  }

private:
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
      for (const int controller_index : controller_indices) {
        if (controller_index < 0 || controller_index > 255) {
          throw std::runtime_error("Robot controller_index is out of uint8 range.");
        }
        controller_indices_.push_back(static_cast<uint8_t>(controller_index));
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
                "motion_control_teach supports one motion_data_file_path per robot config.");
      }
      configured_motion_path = motion_path;
    }

    if (controller_indices_.empty()) {
      throw std::runtime_error("No controller_indices were found in " + config_file);
    }
    std::sort(controller_indices_.begin(), controller_indices_.end());
    controller_indices_.erase(
      std::unique(controller_indices_.begin(), controller_indices_.end()),
      controller_indices_.end());

    motion_record_path_ = configured_motion_path;
  }

  void motor_status_callback(const MotorStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(recording_mutex_);
    if (recording_active_) {
      append_motion_record_sample(*msg);
    }
  }

  void publish_torque_request(int8_t value)
  {
    const std::size_t row_count = controller_indices_.back() + 1;
    Int8MultiArray request;
    request.data.assign(row_count, kNoChange);
    for (const uint8_t controller_index : controller_indices_) {
      request.data[controller_index] = value;
    }
    request_pub_->publish(request);
  }

  void handle_torque_off(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
  {
    publish_torque_request(kTorqueOff);
    response->success = true;
    response->message = "Torque off requested for " +
      std::to_string(controller_indices_.size()) + " controller(s).";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_torque_on(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
  {
    publish_torque_request(kTorqueOn);
    response->success = true;
    response->message = "Torque on requested for " +
      std::to_string(controller_indices_.size()) + " controller(s).";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_start_recording(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lk(recording_mutex_);
    if (recording_active_) {
      response->success = false;
      response->message = "Recording is already in progress.";
      return;
    }
    motion_recording_rows_.clear();
    motion_recording_start_time_ = std::chrono::steady_clock::now();
    recording_active_ = true;
    response->success = true;
    response->message = "Recording started: " + motion_record_path_.string();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_stop_recording(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
  {
    std::vector<std::vector<double>> rows;
    {
      std::lock_guard<std::mutex> lk(recording_mutex_);
      if (!recording_active_) {
        response->success = false;
        response->message = "No recording is in progress.";
        return;
      }
      recording_active_ = false;
      rows.swap(motion_recording_rows_);
    }

    if (rows.empty() || rows.front().empty()) {
      response->success = false;
      response->message = "Recording stopped, but no position samples were captured.";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!save_motion_record(rows)) {
      response->success = false;
      response->message = "Failed to write motion record to " + motion_record_path_.string();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    const double recorded_duration = rows.front().back();
    response->success = true;
    response->message = "Recording saved: " + motion_record_path_.string() +
      " (" + std::to_string(recorded_duration) + " s).";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void append_motion_record_sample(const MotorStatus & status)
  {
    const std::size_t row_count = controller_indices_.back() + 1;

    std::vector<double> sample(row_count, 0.0);
    std::vector<bool> has_sample(row_count, false);
    bool has_recorded_value = false;
    for (const uint8_t controller_index : controller_indices_) {
      double position = 0.0;
      if (!position_for_controller(status, controller_index, position)) {
        continue;
      }
      sample[controller_index] = position;
      has_sample[controller_index] = true;
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

  bool save_motion_record(const std::vector<std::vector<double>> & rows) const
  {
    std::error_code ec;
    const std::filesystem::path directory = motion_record_path_.parent_path();
    if (!directory.empty()) {
      std::filesystem::create_directories(directory, ec);
    }
    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create motion record directory '%s': %s",
        directory.string().c_str(), ec.message().c_str());
      return false;
    }

    if (!write_motion_csv(motion_record_path_, rows)) {
      return false;
    }

    const double recorded_duration =
      (!rows.empty() && !rows.front().empty()) ? rows.front().back() : 0.0;
    if (!update_robot_move_duration(recorded_duration)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to update move_duration in '%s'.",
        robot_config_path_.string().c_str());
    }

    return true;
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
    temporary_path += ".motion_control_teach.tmp";
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

  static bool position_for_controller(
    const MotorStatus & status,
    uint8_t controller_index,
    double & position)
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

  rclcpp::Subscription<MotorStatus>::SharedPtr motor_status_sub_;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr request_pub_;
  rclcpp::Service<Trigger>::SharedPtr torque_off_srv_;
  rclcpp::Service<Trigger>::SharedPtr torque_on_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_recording_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_recording_srv_;

  std::vector<uint8_t> controller_indices_;
  std::filesystem::path motion_record_path_;
  std::filesystem::path robot_config_path_;

  std::mutex recording_mutex_;
  std::vector<std::vector<double>> motion_recording_rows_;
  std::chrono::steady_clock::time_point motion_recording_start_time_{};
  bool recording_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotionControlTeachNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_control_teach_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
