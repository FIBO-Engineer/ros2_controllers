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

#include "brakable_steering_controller/brakable_steering_controller.hpp"

namespace brakable_steering_controller
{
BrakableSteeringController::BrakableSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void BrakableSteeringController::initialize_implementation_parameter_listener()
{
  brakable_param_listener_ =
    std::make_shared<brakable_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn BrakableSteeringController::configure_odometry()
{
  brakable_params_ = brakable_param_listener_->get_params();

  const double wheelbase = brakable_params_.wheelbase;
  const double front_wheel_radius = brakable_params_.front_wheel_radius;
  const double rear_wheel_radius = brakable_params_.rear_wheel_radius;

  if (params_.front_steering)
  {
    odometry_.set_wheel_params(rear_wheel_radius, wheelbase);
  }
  else
  {
    odometry_.set_wheel_params(front_wheel_radius, wheelbase);
  }

  odometry_.set_odometry_type(steering_odometry::BRAKABLE_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "brakable odometry configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BrakableSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double traction_wheel_value = state_interfaces_[STATE_TRACTION_WHEEL].get_value();
    const double steering_position = state_interfaces_[STATE_STEER_AXIS].get_value();

    if (std::isfinite(traction_wheel_value) && std::isfinite(steering_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(traction_wheel_value, steering_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(traction_wheel_value, steering_position, period.seconds());
      }
    }
  }
  return true;
}
}  // namespace brakable_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  brakable_steering_controller::BrakableSteeringController,
  controller_interface::ChainableControllerInterface)
