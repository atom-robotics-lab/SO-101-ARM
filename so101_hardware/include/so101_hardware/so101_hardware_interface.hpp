#ifndef SO101_HARDWARE__SO101_HARDWARE_INTERFACE_HPP_
#define SO101_HARDWARE__SO101_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace so101_hardware
{

class SO101HardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_bridge_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;  
  std::vector<double> hw_commands_; 

  std::mutex state_mutex_;
  std::map<std::string, double> latest_state_by_name_;
  bool received_first_state_{false};

  // Internal ROS2 node + background executor thread talking to the bridge.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commands_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::string commands_topic_{"/so101/hardware_commands"};
  std::string states_topic_{"/so101/hardware_states"};
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__SO101_HARDWARE_INTERFACE_HPP_