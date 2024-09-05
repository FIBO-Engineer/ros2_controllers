// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Daniel Azanov, Dr. Denis
//

#ifndef BRAKABLE_VELOCITY_CONTROLLER__BRAKABLE_VELOCITY_CONTROLLER_HPP_
#define BRAKABLE_VELOCITY_CONTROLLER__BRAKABLE_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "brakable_velocity_controller/visibility_control.h"
#include "brakable_velocity_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"

#include "control_msgs/msg/pid_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace brakable_velocity_controller
{

enum class feedforward_mode_type : std::uint8_t
{
  OFF = 0,
  ON = 1,
};

class BrakableVelocityController : public controller_interface::ChainableControllerInterface
{
public:
  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  BrakableVelocityController();

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;
  using ControllerMeasuredStateMsg = control_msgs::msg::MultiDOFCommand;
  using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

protected:
  controller_interface::return_type update_reference_from_subscribers() override;

  std::shared_ptr<brakable_velocity_controller::ParamListener> param_listener_;
  brakable_velocity_controller::Params params_;

  std::vector<double> measured_state_values_;

  using PidPtr = std::shared_ptr<control_toolbox::PidROS>;
  PidPtr pid_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<ControllerMeasuredStateMsg>::SharedPtr measured_state_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerMeasuredStateMsg>> measured_state_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  // internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

private:
  // callback for topic interface
  BRAKABLE_VELOCITY_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace brakable_velocity_controller

#endif  // BRAKABLE_VELOCITY_CONTROLLER__BRAKABLE_VELOCITY_CONTROLLER_HPP_
