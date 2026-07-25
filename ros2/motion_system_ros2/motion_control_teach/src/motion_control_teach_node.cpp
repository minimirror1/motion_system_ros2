#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <motion_control_msgs/msg/motor_status.hpp>
#include <motion_control_msgs/srv/list_motion_files.hpp>
#include <motion_control_msgs/srv/set_active_motion.hpp>
#include <motion_control_msgs/srv/start_recording.hpp>
#include <motion_control_msgs/srv/stop_recording.hpp>
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

std::string to_lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

bool has_csv_extension(const std::filesystem::path & path)
{
  return to_lower(path.extension().string()) == ".csv";
}

}  // namespace

class MotionControlTeachNode : public rclcpp::Node
{
public:
  using MotorStatus = motion_control_msgs::msg::MotorStatus;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;
  using Trigger = std_srvs::srv::Trigger;
  using StartRecording = motion_control_msgs::srv::StartRecording;
  using StopRecording = motion_control_msgs::srv::StopRecording;
  using ListMotionFiles = motion_control_msgs::srv::ListMotionFiles;
  using SetActiveMotion = motion_control_msgs::srv::SetActiveMotion;

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
    start_recording_srv_ = create_service<StartRecording>(
      "~/start_recording",
      std::bind(
        &MotionControlTeachNode::handle_start_recording, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = create_service<StopRecording>(
      "~/stop_recording",
      std::bind(
        &MotionControlTeachNode::handle_stop_recording, this,
        std::placeholders::_1, std::placeholders::_2));
    list_motion_files_srv_ = create_service<ListMotionFiles>(
      "~/list_motion_files",
      std::bind(
        &MotionControlTeachNode::handle_list_motion_files, this,
        std::placeholders::_1, std::placeholders::_2));
    set_active_motion_srv_ = create_service<SetActiveMotion>(
      "~/set_active_motion",
      std::bind(
        &MotionControlTeachNode::handle_set_active_motion, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "motion_control_teach_node ready: %zu controllers, motion directory '%s'.",
      controller_indices_.size(), config_directory_.string().c_str());
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

    config_directory_ = robot_config_path_.parent_path();
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
        config_directory_);
      if (!has_csv_extension(motion_path)) {
        motion_path /= robot_name + ".csv";
      }
      motion_path = motion_path.lexically_normal();
      if (!configured_motion_path.empty() && configured_motion_path != motion_path) {
        throw std::runtime_error(
                "motion_control_teach supports one motion_data_file_path per robot config.");
      }
      configured_motion_path = motion_path;
      robot_names_.push_back(robot_name);
    }

    if (controller_indices_.empty()) {
      throw std::runtime_error("No controller_indices were found in " + config_file);
    }
    std::sort(controller_indices_.begin(), controller_indices_.end());
    controller_indices_.erase(
      std::unique(controller_indices_.begin(), controller_indices_.end()),
      controller_indices_.end());
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

  // Rejects path traversal, keeps [A-Za-z0-9._-], enforces a ".csv" extension.
  std::optional<std::string> sanitize_file_name(
    const std::string & raw, std::string & error) const
  {
    std::string name = raw;
    const auto is_space = [](unsigned char c) {return std::isspace(c) != 0;};
    name.erase(name.begin(), std::find_if_not(name.begin(), name.end(), is_space));
    name.erase(std::find_if_not(name.rbegin(), name.rend(), is_space).base(), name.end());

    if (name.empty()) {
      error = "File name is empty.";
      return std::nullopt;
    }
    if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
      error = "File name must not contain path separators.";
      return std::nullopt;
    }
    if (name.front() == '.') {
      error = "File name must not start with '.'.";
      return std::nullopt;
    }

    for (char & c : name) {
      const unsigned char uc = static_cast<unsigned char>(c);
      if (std::isalnum(uc) == 0 && c != '.' && c != '_' && c != '-') {
        c = '_';
      }
    }

    if (!has_csv_extension(std::filesystem::path(name))) {
      name += ".csv";
    }
    if (std::filesystem::path(name).stem().string().empty()) {
      error = "File name is empty after sanitization.";
      return std::nullopt;
    }
    return name;
  }

  std::string resolve_unique(const std::string & name) const
  {
    if (!std::filesystem::exists(config_directory_ / name)) {
      return name;
    }
    const std::filesystem::path path(name);
    const std::string stem = path.stem().string();
    const std::string extension = path.extension().string();
    for (int suffix = 2; suffix < 1000; ++suffix) {
      const std::string candidate = stem + "_" + std::to_string(suffix) + extension;
      if (!std::filesystem::exists(config_directory_ / candidate)) {
        return candidate;
      }
    }
    return stem + "_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + extension;
  }

  std::string make_auto_file_name() const
  {
    const std::time_t now = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    std::tm tm_buffer{};
    localtime_r(&now, &tm_buffer);  // host local time (KST on the target system)

    char minute_name[32];
    std::strftime(minute_name, sizeof(minute_name), "teach_%Y%m%d_%H%M.csv", &tm_buffer);
    if (!std::filesystem::exists(config_directory_ / minute_name)) {
      return minute_name;
    }

    char second_name[32];
    std::strftime(second_name, sizeof(second_name), "teach_%Y%m%d_%H%M%S.csv", &tm_buffer);
    return resolve_unique(second_name);
  }

  void handle_start_recording(
    const std::shared_ptr<StartRecording::Request> request,
    std::shared_ptr<StartRecording::Response> response)
  {
    std::lock_guard<std::mutex> lk(recording_mutex_);
    if (recording_active_) {
      response->success = false;
      response->message = "Recording is already in progress.";
      return;
    }

    std::string name;
    if (request->file_name.empty()) {
      name = make_auto_file_name();
    } else {
      std::string error;
      const auto sanitized = sanitize_file_name(request->file_name, error);
      if (!sanitized) {
        response->success = false;
        response->message = error;
        return;
      }
      name = resolve_unique(*sanitized);
    }

    motion_record_path_ = config_directory_ / name;
    motion_recording_rows_.clear();
    motion_recording_start_time_ = std::chrono::steady_clock::now();
    recording_active_ = true;
    response->success = true;
    response->file_name = name;
    response->message = "Recording started: " + motion_record_path_.string();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_stop_recording(
    const std::shared_ptr<StopRecording::Request>,
    std::shared_ptr<StopRecording::Response> response)
  {
    std::vector<std::vector<double>> rows;
    std::filesystem::path record_path;
    {
      std::lock_guard<std::mutex> lk(recording_mutex_);
      if (!recording_active_) {
        response->success = false;
        response->message = "No recording is in progress.";
        return;
      }
      recording_active_ = false;
      rows.swap(motion_recording_rows_);
      record_path = motion_record_path_;
    }

    if (rows.empty() || rows.front().empty()) {
      response->success = false;
      response->message = "Recording stopped, but no position samples were captured.";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const std::string file_name = record_path.filename().string();
    const double recorded_duration = rows.front().back();

    std::error_code ec;
    std::filesystem::create_directories(record_path.parent_path(), ec);
    if (ec || !write_motion_csv(record_path, rows)) {
      response->success = false;
      response->message = "Failed to write motion record to " + record_path.string();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    response->file_name = file_name;
    response->duration = recorded_duration;

    if (!update_robot_config(recorded_duration, file_name)) {
      response->success = false;
      response->message = "Recording saved to " + file_name +
        " but failed to update active config.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Recording saved and activated: " + file_name +
      " (" + std::to_string(recorded_duration) + " s).";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handle_list_motion_files(
    const std::shared_ptr<ListMotionFiles::Request>,
    std::shared_ptr<ListMotionFiles::Response> response)
  {
    std::error_code ec;
    std::filesystem::directory_iterator it(config_directory_, ec);
    if (ec) {
      response->success = false;
      response->message = "Motion directory not found: " + config_directory_.string();
      return;
    }

    for (const auto & entry : it) {
      if (entry.is_regular_file() && has_csv_extension(entry.path())) {
        response->files.push_back(entry.path().filename().string());
      }
    }
    std::sort(response->files.begin(), response->files.end());

    response->success = true;
    response->message = std::to_string(response->files.size()) + " motion file(s).";
    try {
      response->active_file = read_active_motion_file();
    } catch (const std::exception & e) {
      response->active_file = "";
      response->message += " Failed to read active file: " + std::string(e.what());
    }
  }

  void handle_set_active_motion(
    const std::shared_ptr<SetActiveMotion::Request> request,
    std::shared_ptr<SetActiveMotion::Response> response)
  {
    std::string error;
    const auto sanitized = sanitize_file_name(request->file_name, error);
    if (!sanitized) {
      response->success = false;
      response->message = error;
      return;
    }
    const std::string name = *sanitized;
    const std::filesystem::path path = config_directory_ / name;
    if (!std::filesystem::is_regular_file(path)) {
      response->success = false;
      response->message = "File not found: " + path.string();
      return;
    }

    const std::optional<double> duration = read_motion_duration(path);
    if (!update_robot_config(duration, name)) {
      response->success = false;
      response->message = "Failed to update active config for " + name;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Active motion set to " + name +
      (duration ? "" : " (no time row found, move_duration unchanged)");
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  // The active yaml changes at runtime now, so always re-read it from disk.
  std::string read_active_motion_file() const
  {
    const YAML::Node root = YAML::LoadFile(robot_config_path_.string());
    const YAML::Node robots = root["robot"];
    if (!robots || !robots.IsSequence() || robots.size() == 0) {
      throw std::runtime_error("Invalid 'robot' in " + robot_config_path_.string());
    }
    std::filesystem::path motion_path = resolve_resource_path(
      required_as<std::string>(robots[0], "motion_data_file_path", "robot[]"),
      config_directory_);
    if (!has_csv_extension(motion_path) && !robot_names_.empty()) {
      motion_path /= robot_names_.front() + ".csv";
    }
    return motion_path.filename().string();
  }

  // Row 0 is a time axis only when the CSV has more rows than controllers,
  // mirroring robot.py::_load_motion_data; otherwise the duration is unknown.
  std::optional<double> read_motion_duration(const std::filesystem::path & path) const
  {
    std::ifstream input(path);
    if (!input) {
      return std::nullopt;
    }

    std::string first_line;
    std::size_t line_count = 0;
    std::string line;
    while (std::getline(input, line)) {
      if (line.find_first_not_of(" \t\r,") == std::string::npos) {
        continue;
      }
      if (line_count == 0) {
        first_line = line;
      }
      ++line_count;
    }

    if (line_count < controller_indices_.size() + 1) {
      return std::nullopt;
    }

    const std::size_t last_comma = first_line.find_last_of(',');
    const std::string last_value =
      last_comma == std::string::npos ? first_line : first_line.substr(last_comma + 1);
    try {
      const double duration = std::stod(last_value);
      if (!std::isfinite(duration) || duration < 0.0) {
        return std::nullopt;
      }
      return duration;
    } catch (const std::exception &) {
      return std::nullopt;
    }
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

  bool update_robot_config(
    std::optional<double> duration,
    const std::optional<std::string> & motion_file) const
  {
    if (robot_config_path_.empty()) {
      return false;
    }

    YAML::Node root;
    YAML::Node robots;
    try {
      root = YAML::LoadFile(robot_config_path_.string());
      robots = root["robot"];
    } catch (const std::exception &) {
      return false;
    }
    if (!robots || !robots.IsSequence() || robots.size() == 0) {
      return false;
    }

    for (std::size_t i = 0; i < robots.size(); ++i) {
      if (duration) {
        robots[i]["move_duration"] = std::round(*duration * 10.0) / 10.0;
      }
      if (motion_file) {
        robots[i]["motion_data_file_path"] = *motion_file;
      }
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
  rclcpp::Service<StartRecording>::SharedPtr start_recording_srv_;
  rclcpp::Service<StopRecording>::SharedPtr stop_recording_srv_;
  rclcpp::Service<ListMotionFiles>::SharedPtr list_motion_files_srv_;
  rclcpp::Service<SetActiveMotion>::SharedPtr set_active_motion_srv_;

  std::vector<uint8_t> controller_indices_;
  std::vector<std::string> robot_names_;
  std::filesystem::path config_directory_;
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
