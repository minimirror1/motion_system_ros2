#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

#include "common_motor_interface/motor_frame.hpp"

#include "motion_control_bridge/motor_manager_node.hpp"
#include "motion_control_bridge/status_publish_policy.hpp"

namespace
{

std::string read_binary_file(const char * path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return {};
  }

  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>(),
  };
}

std::string printable_device_model(std::string model)
{
  const auto null_position = model.find('\0');
  if (null_position != std::string::npos) {
    model.resize(null_position);
  }

  while (!model.empty() &&
    (model.back() == '\n' || model.back() == '\r' || model.back() == ' '))
  {
    model.pop_back();
  }

  return model.empty() ? "not reported" : model;
}

}  // namespace

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions & options)
: Node("motor_manager_node", options)
{
  motor_command_subscriber_ = this->create_subscription<MotorStatus>(
    "motion_control/motor_command", rclcpp::QoS(1).best_effort(),
    [this](const MotorStatus::SharedPtr msg) {
      motor_command_callback(msg);
    });

  request_subscriber_ = this->create_subscription<Int8MultiArray>(
    "motion_control/request", rclcpp::QoS(1).best_effort(),
    [this](const Int8MultiArray::SharedPtr msg) {
      request_callback(msg);
    });

  motor_status_publisher_ = this->create_publisher<MotorStatus>(
    "motion_control/motor_status", rclcpp::QoS(1).best_effort());

  config_file_ = this->declare_parameter<std::string>("config_file", "");
  if (config_file_.empty()) {
    throw std::runtime_error(
            "Parameter 'config_file' is empty. Use e.g. "
            "`ros2 launch motion_control_bridge motor_manager_node.launch.py`.");
  }

  const bool jog_mode = this->declare_parameter<bool>("jog_mode", false);
  const auto runtime_profile_value =
    this->declare_parameter<std::string>("runtime_profile", "auto");
  const auto requested_status_rate_hz =
    this->declare_parameter<std::int64_t>("motor_status_publish_rate_hz", 0);

  const auto device_tree_model =
    read_binary_file("/proc/device-tree/model");
  const auto device_tree_compatible =
    read_binary_file("/proc/device-tree/compatible");
  const auto cpu_info = read_binary_file("/proc/cpuinfo");
  const bool raspberry_pi = motion_control_bridge::is_raspberry_pi(
    device_tree_model,
    device_tree_compatible,
    cpu_info);
  const auto runtime_profile =
    motion_control_bridge::parse_runtime_profile(runtime_profile_value);
  const int status_publish_rate_hz =
    motion_control_bridge::select_status_publish_rate_hz(
    runtime_profile,
    requested_status_rate_hz,
    raspberry_pi);
  const auto status_publish_period = std::chrono::nanoseconds(
    (1'000'000'000LL + status_publish_rate_hz / 2) /
    status_publish_rate_hz);

  motor_manager_ = std::make_unique<motor_manager::MotorManager>(config_file_, jog_mode);

  motor_status_timer_ = this->create_wall_timer(
    status_publish_period,
    [this]() {
      timer_callback();
    });

  std::string rate_source;
  if (requested_status_rate_hz > 0) {
    rate_source = "explicit motor_status_publish_rate_hz";
  } else if (runtime_profile == motion_control_bridge::RuntimeProfile::Auto) {
    rate_source = raspberry_pi ?
      "automatic Raspberry Pi profile" :
      "automatic desktop profile";
  } else {
    rate_source = "explicit " +
      std::string(motion_control_bridge::runtime_profile_name(runtime_profile)) +
      " profile";
  }

  RCLCPP_INFO(
    get_logger(),
    "Platform model: %s; Raspberry Pi: %s; runtime_profile: %s",
    printable_device_model(device_tree_model).c_str(),
    raspberry_pi ? "yes" : "no",
    std::string(
      motion_control_bridge::runtime_profile_name(runtime_profile)
    ).c_str());
  RCLCPP_INFO(
    get_logger(),
    "motor_status publish rate: %d Hz (%s); motor control period: %u ns (unchanged)",
    status_publish_rate_hz,
    rate_source.c_str(),
    motor_manager_->period());

  manager_run_thread_ = std::thread([this]() {
        try {
          motor_manager_->run();
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "MotorManager::run() failed: %s", e.what());
        }
  });
}

MotorManagerNode::~MotorManagerNode()
{
  if (motor_manager_) {
    motor_manager_->request_exit();
  }
  if (manager_run_thread_.joinable()) {
    manager_run_thread_.join();
  }
}

void MotorManagerNode::motor_command_callback(const MotorStatus::SharedPtr msg)
{
  const size_t size = std::min({
    msg->controller_index.size(),
    static_cast<size_t>(motor_interface::MAX_CONTROLLER_SIZE),
  });

  motor_interface::motor_frame_t motor_frame[motor_interface::MAX_CONTROLLER_SIZE] = {};

  for (uint8_t i = 0; i < static_cast<uint8_t>(size); i++) {
    const size_t requested_if_count = i < msg->number_of_target_interfaces.size() ?
      static_cast<size_t>(msg->number_of_target_interfaces[i]) : 0;
    const size_t target_data_count = i < msg->target_interface_id.size() ?
      msg->target_interface_id[i].data.size() : 0;
    const size_t n_if = std::min({
      requested_if_count,
      target_data_count,
      static_cast<size_t>(motor_interface::MAX_INTERFACE_SIZE),
    });
    motor_frame[i].number_of_target_interfaces = static_cast<uint8_t>(n_if);

    for (uint8_t j = 0; j < n_if; j++) {
      motor_frame[i].target_interface_id[j] = msg->target_interface_id[i].data[j];
    }
    motor_frame[i].controller_index = msg->controller_index[i];
    if (i < msg->controlword.size()) {
      motor_frame[i].controlword = msg->controlword[i];
    }
    // motor_frame[i].statusword = msg->statusword[i];
    // motor_frame[i].errorcode = msg->errorcode[i];
    if (i < msg->encoder.size()) {
      motor_frame[i].encoder = msg->encoder[i];
    }
    if (i < msg->position.size()) {
      motor_frame[i].position = msg->position[i];
    }
    if (i < msg->velocity.size()) {
      motor_frame[i].velocity = msg->velocity[i];
    }
    if (i < msg->effort.size()) {
      motor_frame[i].effort = msg->effort[i];
    }
  }

  motor_manager_->write(motor_frame, static_cast<uint8_t>(size));
}

void MotorManagerNode::request_callback(const Int8MultiArray::SharedPtr msg)
{
  const size_t size = std::min({
    msg->data.size(),
    static_cast<size_t>(motor_interface::MAX_CONTROLLER_SIZE),
  });
  motor_manager_->request(msg->data.data(), static_cast<uint8_t>(size));
}

void MotorManagerNode::timer_callback()
{
  const uint8_t n = motor_manager_->number_of_controllers();
  if (n == 0) {
    return;
  }

  motor_interface::motor_frame_t status[motor_interface::MAX_CONTROLLER_SIZE] = {};
  motor_manager_->read(status);

  MotorStatus msg;
  msg.number_of_target_interfaces.resize(n);
  msg.controller_index.resize(n);
  msg.controlword.resize(n);
  msg.statusword.resize(n);
  msg.errorcode.resize(n);
  msg.encoder.resize(n);
  msg.position.resize(n);
  msg.velocity.resize(n);
  msg.effort.resize(n);

  for (uint8_t i = 0; i < n; i++) {
    msg.controller_index[i] = status[i].controller_index;
    msg.controlword[i] = status[i].controlword;
    msg.statusword[i] = status[i].statusword;
    msg.errorcode[i] = status[i].errorcode;
    msg.encoder[i] = status[i].encoder;
    msg.position[i] = status[i].position;
    msg.velocity[i] = status[i].velocity;
    msg.effort[i] = status[i].effort;
  }

  motor_status_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorManagerNode>());
  rclcpp::shutdown();
  return 0;
}
