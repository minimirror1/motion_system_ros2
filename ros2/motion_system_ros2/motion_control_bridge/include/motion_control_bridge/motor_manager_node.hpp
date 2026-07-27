#ifndef MOTOR_MANAGER_NODE_HPP_
#define MOTOR_MANAGER_NODE_HPP_

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "motion_control_msgs/msg/motor_status.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "motor_manager/motor_manager.hpp"

class MotorManagerNode : public rclcpp::Node {
public:
  using MotorStatus = motion_control_msgs::msg::MotorStatus;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;

  explicit MotorManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~MotorManagerNode();

private:
  void motor_command_callback(const MotorStatus::SharedPtr msg);

  void request_callback(const Int8MultiArray::SharedPtr msg);

  void timer_callback();

  rclcpp::Subscription<MotorStatus>::SharedPtr motor_command_subscriber_;

  rclcpp::Subscription<Int8MultiArray>::SharedPtr request_subscriber_;

  rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_publisher_;

  rclcpp::TimerBase::SharedPtr motor_status_timer_;

  std::string config_file_;

  std::unique_ptr<motor_manager::MotorManager> motor_manager_;

  std::thread manager_run_thread_;
};

#endif // MOTOR_MANAGER_NODE_HPP_
