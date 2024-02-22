// Copyright (c) 2024, ICube
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "secured_1d_velocity_controller/secured_1d_velocity_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg =
  secured_1d_velocity_controller::Secured1dVelocityController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(std::shared_ptr<ControllerReferenceMsg> & msg)
{
  /*
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  */
  (*msg).data = 0.0;
}

}  // namespace

namespace secured_1d_velocity_controller
{
Secured1dVelocityController::Secured1dVelocityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn Secured1dVelocityController::on_init()
{
  // declare and get parameters needed for controller initialization
  // allocate memory that will exist for the life of the controller
  control_mode_.initRT(control_mode_type::LIMITS_DISCOVERY);

  try
  {
    param_listener_ = std::make_shared<secured_1d_velocity_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Secured1dVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /*
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Commmand Velocity Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference_velocity", subscribers_qos,
    std::bind(&Secured1dVelocityController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_velocity_sign_security_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_velocity_sign_security_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_velocity_sign_security_mode", set_velocity_sign_security_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();
  */

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Commmand Velocity Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference_velocity", subscribers_qos,
    std::bind(&Secured1dVelocityController::reference_callback, this, std::placeholders::_1));

  // Default command value is the 0.0 velocity (no motion)
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  (*msg).data = 0.0;
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void Secured1dVelocityController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  /*
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
  */
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration
Secured1dVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  /*
  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  */
  command_interfaces_config.names.reserve(1);
  std::string joint("base_to_charriot");
  command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
Secured1dVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  /**
  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }
  */

  /** Motor states */
  state_interfaces_config.names.reserve(5);
  std::string joint("base_to_charriot");
  state_interfaces_config.names.push_back(
    joint + std::string("/") + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(
    joint + std::string("/") + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(
    joint + std::string("/") + hardware_interface::HW_IF_EFFORT);

  /** Limit switch states*/
  state_interfaces_config.names.push_back(
    "switch1_contact_sensor"
    "/"
    "limit_switch_left");
  state_interfaces_config.names.push_back(
    "switch2_contact_sensor"
    "/"
    "limit_switch_right");

  return state_interfaces_config;
}

controller_interface::CallbackReturn Secured1dVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for examplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Define state interface indices for limit switches
  limit_switch0_idx_ = 3;
  limit_switch1_idx_ = 4;

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Secured1dVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

double Secured1dVelocityController::discover_limits__limit_reached_command(
  const std::size_t limit_index, const double reference_velocity, const double current_velocity,
  bool & blocked)
{
  // 2 cases: either it is the first time limit is reached (UNKNOWN) or not (KNOWN)

  // Test KNOWN cases
  if (discovery_state_.start_discovered && limit_index == discovery_state_.start_idx)
  {
    // We are in the case where the limit has already been reached
    // We can check if the velocity is admissible
    return limit_map_.admissible_velocity(limit_index, reference_velocity, blocked);
  }

  if (discovery_state_.end_discovered && limit_index == discovery_state_.end_idx)
  {
    // We are in the case where the limit has already been reached
    // We can check if the velocity is admissible
    return limit_map_.admissible_velocity(limit_index, reference_velocity, blocked);
  }

  // Handle UNKNOWN case
  blocked = true;  // We block the velocity
  const auto current_vel_mag = std::abs(current_velocity);
  if (current_vel_mag < zero_velocity_tolerance_)
  {
    // We are in the case where the velocity is 0
    // We cannot learn the limit position because velocity direction can be false due to noise
    // We don't move
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "We cannot make limit discovery with a too small velocity (%f). Please, manually set the "
      "system in a non limit position.",
      current_velocity);
    return 0.;
  }

  if (current_velocity > 0.)
  {
    // We discovered the end limit
    discovery_state_.end_discovered = true;
    discovery_state_.end_idx = limit_index;
    limit_map_.end_siidx = limit_index;
    RCLCPP_INFO(
      get_node()->get_logger(), "Discovered end limit, interface index %zu with velocity %f",
      limit_index, current_velocity);
  }
  else
  {
    // We discovered the start limit
    discovery_state_.start_discovered = true;
    discovery_state_.start_idx = limit_index;
    limit_map_.start_siidx = limit_index;
    RCLCPP_INFO(
      get_node()->get_logger(), "Discovered start limit, interface index %zu with velocity %f",
      limit_index, current_velocity);
  }

  if (discovery_state_.discovery_is_done())
  {
    control_mode_.writeFromNonRT(control_mode_type::LIMITS_DISCOVERED);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "All limits discovered: change control_mode_type to LIMITS_DISCOVERED");
  }
  return 0.;
}

inline void Secured1dVelocityController::record_limit_switch_state(
  bool limit_left_active, bool limit_right_active)
{
  // Here we make some log
  if (
    limit_switch_left_prev_active_state_ != limit_left_active ||
    limit_switch_right_prev_active_state_ != limit_right_active)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Limit switch state changed: left: (%d->%d), right: (%d->%d)",
      limit_switch_left_prev_active_state_, limit_left_active,
      limit_switch_right_prev_active_state_, limit_right_active);
  }
  limit_switch_left_prev_active_state_ = limit_left_active;
  limit_switch_right_prev_active_state_ = limit_right_active;
}

inline void Secured1dVelocityController::log_blocked_velocity_command(
  const bool block, const double current_ref, const double current_velocity)
{
  auto current_vel_mag = std::abs(current_velocity);
  if (current_vel_mag < zero_velocity_tolerance_)
  {
    return;
  }

  if (block && !velocity_reference_cmd_is_blocked_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Blocked velocity command %f because of limit switch (current_velocity was %f)", current_ref,
      current_velocity);
    velocity_reference_cmd_is_blocked_ = true;
  }
  else
  {
    if (!block && velocity_reference_cmd_is_blocked_)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Unblocked velocity command %f (current_velocity was %f)",
        current_ref, current_velocity);
      velocity_reference_cmd_is_blocked_ = false;
    }
  }
}

controller_interface::return_type Secured1dVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref_msg = input_ref_.readFromRT();

  if (!current_ref_msg)
  {  // ERROR if no velocity command received
    RCLCPP_ERROR(get_node()->get_logger(), "No velocity command received.");
    return controller_interface::return_type::ERROR;
  }

  const auto current_ref = (*current_ref_msg)->data;
  const auto current_velocity = state_interfaces_[0].get_value();
  const auto current_position = state_interfaces_[1].get_value();
  const auto current_effort = state_interfaces_[2].get_value();
  const auto current_limit_switch0 = state_interfaces_[3].get_value();
  const auto current_limit_switch1 = state_interfaces_[4].get_value();

  const bool limit0_active = (current_limit_switch0 != 0);
  const bool limit1_active = (current_limit_switch1 != 0);

  if (!std::isnan(current_ref))
  {  // Velocity reference command is well defined
    const auto ref_vel_mag = std::abs(current_ref);
    // if (0.0 == current_ref)
    if (ref_vel_mag < zero_velocity_tolerance_)
    {  // Velocity reference command is 0.0, set velocity to 0.0
      command_interfaces_[0].set_value(0.);
      return controller_interface::return_type::OK;
    }

    if (limit0_active && limit1_active)
    {
      // Both limit switches are active (that is very bad!), set velocity to 0.0
      RCLCPP_ERROR(get_node()->get_logger(), "Both limit switches are active");
      command_interfaces_[0].set_value(0.);
      record_limit_switch_state(limit0_active, limit1_active);
      return controller_interface::return_type::OK;
    }

    if (!limit0_active && !limit1_active)
    {
      // Limit switches are inactive, set velocity to velocity reference command
      command_interfaces_[0].set_value(current_ref);
      record_limit_switch_state(limit0_active, limit1_active);
      // Log the fact that we unblocked the velocity
      log_blocked_velocity_command(false, current_ref, current_velocity);
      return controller_interface::return_type::OK;
    }
    else
    {
      std::size_t limit_idx = 666666;
      if (limit0_active)
      {
        limit_idx = limit_switch0_idx_;
      }
      else
      {
        limit_idx = limit_switch1_idx_;
      }

      bool blocked = false;
      double vel_cmd = std::numeric_limits<double>::quiet_NaN();

      // 1 limit switch is active, only move in an admissible direction
      // if the limits are known, we can check if the velocity is admissible
      if (control_mode_type::LIMITS_DISCOVERED == *(control_mode_.readFromRT()))
      {
        vel_cmd = limit_map_.admissible_velocity(limit_idx, current_ref, blocked);
      }
      else
      {
        // Limits have yet to be mapped
        vel_cmd =
          discover_limits__limit_reached_command(limit_idx, current_ref, current_velocity, blocked);
      }

      command_interfaces_[0].set_value(vel_cmd);
      record_limit_switch_state(limit0_active, limit1_active);
      log_blocked_velocity_command(blocked, current_ref, current_velocity);
    }
  }
  else
  {
    // Velocity reference command is NaN, set velocity to 0.0
    command_interfaces_[0].set_value(0.);
    record_limit_switch_state(limit0_active, limit1_active);
    // Log the fact that we blocked the velocity
    log_blocked_velocity_command(true, current_ref, current_velocity);
    return controller_interface::return_type::OK;
  }

  /**
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      if (*(control_mode_.readFromRT()) == control_mode_type::INFERRED_VELOCITY_SIGN_SECURITY)
      {
        // TODO

        command_interfaces_[i].set_value((*current_ref)->displacements[i]);
      }
      else
      {
        // TODO
      }

      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }
  */

  return controller_interface::return_type::OK;
}

}  // namespace secured_1d_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  secured_1d_velocity_controller::Secured1dVelocityController,
  controller_interface::ControllerInterface)
