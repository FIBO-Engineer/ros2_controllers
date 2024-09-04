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

#include "brakable_velocity_controller/brakable_velocity_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/single_dof_state.hpp"
#include "controller_interface/helpers.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{  // utility

// Changed services history QoS to keep all so we don't lose any client service calls
static const rmw_qos_profile_t qos_services = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerCommandMsg = brakable_velocity_controller::BrakableVelocityController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(const std::shared_ptr<ControllerCommandMsg> & msg)
{
  msg->values.resize(1, std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(1, std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg( const std::shared_ptr<ControllerCommandMsg> & msg)
{
  reset_controller_reference_msg(msg);
}

}  // namespace

namespace brakable_velocity_controller
{
BrakableVelocityController::BrakableVelocityController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn BrakableVelocityController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<brakable_velocity_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void BrakableVelocityController::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn BrakableVelocityController::configure_parameters()
{
  update_parameters();
  // TODO: Can we check if there're actually interfaces that we're looking for?

  // prefix should be interpreted as parameters prefix
  pid_ = std::make_shared<control_toolbox::PidROS>(get_node(), "gains", true);
  if (!pid_->initPid())
  {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BrakableVelocityController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pid_->reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BrakableVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber, which part of code unsubscribe when it's in chain mode 
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&BrakableVelocityController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  std::shared_ptr<ControllerMeasuredStateMsg> measured_state_msg = std::make_shared<ControllerMeasuredStateMsg>();
  reset_controller_measured_state_msg(measured_state_msg);
  measured_state_.writeFromNonRT(measured_state_msg);

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher
  state_publisher_->lock();
  state_publisher_->msg_.dof_states.resize(2);
  state_publisher_->msg_.dof_states[0].name = params_.velocity_feedforward_dof;
  state_publisher_->msg_.dof_states[1].name = params_.brake_pid_dof;
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void BrakableVelocityController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->values.size() == 1)
  {
    input_ref_.writeFromNonRT(msg);
  } else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of input data expected to be only one, just to control the target velocity");
  }
}

controller_interface::InterfaceConfiguration BrakableVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(2);
  command_interfaces_config.names.push_back(params_.velocity_feedforward_dof + "/velocity");
  command_interfaces_config.names.push_back(params_.brake_pid_dof + "/position");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration BrakableVelocityController::state_interface_configuration() const
{
  /* Use just what we need */
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(1);
  // state_interfaces_config.names.reserve(3);
  state_interfaces_config.names.push_back(params_.velocity_feedforward_dof + "/velocity");
  // state_interfaces_config.names.push_back(params_.velocity_feedforward_dof + "/position");
  // state_interfaces_config.names.push_back(params_.brake_pid_dof + "/position");
  
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> BrakableVelocityController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), params_.velocity_feedforward_dof + "/velocity", &reference_interfaces_[0]));

  return reference_interfaces;
}

bool BrakableVelocityController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn BrakableVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_controller_reference_msg(*(input_ref_.readFromRT()));
  reset_controller_measured_state_msg(*(measured_state_.readFromRT()));

  reference_interfaces_.assign(1, std::numeric_limits<double>::quiet_NaN());
  measured_state_values_.assign(1, std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BrakableVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BrakableVelocityController::update_reference_from_subscribers()
{
  auto current_ref = input_ref_.readFromRT();

  if (!std::isnan((*current_ref)->values[0]))
  {
    reference_interfaces_[0] = (*current_ref)->values[0];
    (*current_ref)->values[0] = std::numeric_limits<double>::quiet_NaN();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type BrakableVelocityController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();

  // read measure state
  measured_state_values_[0] = state_interfaces_[0].get_value();

  double brake_command = std::numeric_limits<double>::quiet_NaN();

  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(measured_state_values_[0]))
  {
    brake_command = 0.0;
    double velocity_error = reference_interfaces_[0] - measured_state_values_[0];

    // use calculation with 'velocity_error' only
    brake_command += pid_->computeCommand(velocity_error, period);
    if(brake_command > params_.brake_limits.upper)
      brake_command = params_.brake_limits.upper;
    else if(brake_command < params_.brake_limits.lower)
      brake_command = params_.brake_limits.lower;

    // write calculated values
    command_interfaces_[0].set_value(reference_interfaces_[0]);
    command_interfaces_[1].set_value(brake_command);
  }
  

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;

    state_publisher_->msg_.dof_states[0].reference = reference_interfaces_[0];
    state_publisher_->msg_.dof_states[0].feedback = measured_state_values_[0];
    state_publisher_->msg_.dof_states[0].error = reference_interfaces_[0] - measured_state_values_[0];
    state_publisher_->msg_.dof_states[0].time_step = period.seconds();
    state_publisher_->msg_.dof_states[0].output = command_interfaces_[0].get_value();

    state_publisher_->msg_.dof_states[1].reference = reference_interfaces_[1];
    state_publisher_->msg_.dof_states[1].feedback = 0.0;
    state_publisher_->msg_.dof_states[1].error = 0.0;
    state_publisher_->msg_.dof_states[1].time_step = period.seconds();
    state_publisher_->msg_.dof_states[1].output = command_interfaces_[1].get_value();


    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace brakable_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  brakable_velocity_controller::BrakableVelocityController, controller_interface::ChainableControllerInterface)
