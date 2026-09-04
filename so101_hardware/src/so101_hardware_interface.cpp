#include "so101_hardware/so101_hardware_interface.hpp"

#include <chrono>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace so101_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn SO101HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);

    // This interface only supports a single position command/state pair per
    // joint, matching what the Feetech bus + bridge node provide.
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SO101HardwareInterface"),
        "Joint '%s' must expose exactly one command interface: 'position'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SO101HardwareInterface"),
        "Joint '%s' must expose exactly one state interface: 'position'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Optional URDF <ros2_control> <hardware> <param> overrides for topic names.
  if (info_.hardware_parameters.count("commands_topic")) {
    commands_topic_ = info_.hardware_parameters.at("commands_topic");
  }
  if (info_.hardware_parameters.count("states_topic")) {
    states_topic_ = info_.hardware_parameters.at("states_topic");
  }

  hw_positions_.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

void SO101HardwareInterface::on_bridge_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    latest_state_by_name_[msg->name[i]] = msg->position[i];
  }
  received_first_state_ = true;
}

CallbackReturn SO101HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = rclcpp::Node::make_shared("so101_hardware_interface_node");

  commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(commands_topic_, 10);
  states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    states_topic_, 10,
    std::bind(&SO101HardwareInterface::on_bridge_state, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting for first state message from bridge node on '%s'...", states_topic_.c_str());

  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (received_first_state_) {
        break;
      }
    }
    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Timed out waiting for the feetech_bridge_node on topic '%s'. "
        "Is it running? Is the port open? Refusing to activate.",
        states_topic_.c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Seed both state and command with the arm's actual current position so the
  // controller does not command a jump the instant it starts.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = latest_state_by_name_.find(joint_names_[i]);
      if (it == latest_state_by_name_.end()) {
        RCLCPP_FATAL(
          node_->get_logger(),
          "Bridge node never reported a position for joint '%s'. Check that the "
          "URDF joint names match the lerobot motor names exactly.",
          joint_names_[i].c_str());
        return CallbackReturn::ERROR;
      }
      hw_positions_[i] = it->second;
      hw_commands_[i] = it->second;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "SO101HardwareInterface activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  states_sub_.reset();
  commands_pub_.reset();
  executor_.reset();
  node_.reset();
  received_first_state_ = false;
  return CallbackReturn::SUCCESS;
}

return_type SO101HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = latest_state_by_name_.find(joint_names_[i]);
    if (it != latest_state_by_name_.end()) {
      hw_positions_[i] = it->second;
    }
    // If not found (yet), keep the previous cached value rather than writing NaN,
    // so a single dropped message doesn't fault the controller.
  }
  return return_type::OK;
}

return_type SO101HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.name = joint_names_;
  msg.position = hw_commands_;
  commands_pub_->publish(msg);
  return return_type::OK;
}

}  // namespace so101_hardware

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101HardwareInterface, hardware_interface::SystemInterface)