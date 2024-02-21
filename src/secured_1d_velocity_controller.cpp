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
  control_mode_.initRT(control_mode_type::INFERRED_VELOCITY_SIGN_SECURITY);

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

controller_interface::return_type Secured1dVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref_msg = input_ref_.readFromRT();

  if (!current_ref_msg)
  {  // ERROR if no velocity command received
    RCLCPP_ERROR(get_node()->get_logger(), "No velocity command received.");
    return controller_interface::return_type::ERROR;
  }

  auto current_ref = (*current_ref_msg)->data;

  const auto current_limit_switch_left = state_interfaces_[3].get_value();
  const auto current_limit_switch_right = state_interfaces_[4].get_value();

  const bool limit_left_active = (current_limit_switch_left != 0);
  const bool limit_right_active = (current_limit_switch_right != 0);

  if (!std::isnan(current_ref))
  {  // Velocity reference command is well defined
    const auto ref_vel_mag = std::abs(current_ref);
    // if (0.0 == current_ref)
    if (ref_vel_mag < zero_velocity_tolerance_)
    {  // Velocity reference command is 0.0, set velocity to 0.0
      command_interfaces_[0].set_value(0.0);
      return controller_interface::return_type::OK;
    }

    const auto current_velocity = state_interfaces_[0].get_value();
    const auto current_position = state_interfaces_[1].get_value();
    const auto current_effort = state_interfaces_[2].get_value();

    const auto current_vel_mag = std::abs(current_velocity);

    if (limit_left_active && limit_right_active)
    {
      // Both limit switches are active (that is very bad!), set velocity to 0.0
      RCLCPP_ERROR(get_node()->get_logger(), "Both limit switches are active");
      command_interfaces_[0].set_value(0.0);
      record_limit_switch_state(limit_left_active, limit_right_active);
      return controller_interface::return_type::OK;
    }

    if (limit_left_active || limit_right_active)
    {
      if (current_vel_mag > zero_velocity_tolerance_)
      {
        // Current velocity is not 0
        //=========================

        // 1 limit switch is active, set velocity to 0.0 if velocity reference command has same sign
        // as current velocity
        if (
          (current_velocity > 0. && current_ref > 0.) ||
          (current_velocity < 0. && current_ref < 0.))
        {  // Current velocity and wished velocity (reference command) have the same sign.
           // So we are asked to continue a movement that has already exceeded the limit.
           // So we stop the movement.
          command_interfaces_[0].set_value(0.);

          // Record the velocity sign associated with the limit violated
          if (limit_left_active)
          {  // Left limit switch is active
            limit_switch_left_violating_velocity_sign_ = current_ref > 0. ? 1.0 : -1.0;
            limit_switch_right_violating_velocity_sign_ =
              -limit_switch_left_violating_velocity_sign_;
          }
          else
          {
            // Right limit switch is active
            limit_switch_right_violating_velocity_sign_ = current_ref > 0. ? 1.0 : -1.0;
            limit_switch_left_violating_velocity_sign_ =
              -limit_switch_right_violating_velocity_sign_;
          }
          // Velocity sign associated with the violated limit is recorded
          record_limit_switch_state(limit_left_active, limit_right_active);
          return controller_interface::return_type::OK;
        }
        else
        {
          // Current velocity is not 0 and
          // current velocity and wished velocity (reference command) have different signs.
          // So we are asked to move in the opposite direction of the limit that was exceeded.
          // So we perform the movement as asked.
          command_interfaces_[0].set_value(current_ref);
          record_limit_switch_state(limit_left_active, limit_right_active);
          return controller_interface::return_type::OK;
        }
      }
      else
      {
        // Current velocity is considered 0
        //=================================

        // We don't know the current velocity sign, so we don't move except if we have already
        // learned the authorized velocity signs
        if (limit_left_active)
        {  // switch left is the only active one
          if (limit_switch_left_violating_velocity_sign_ * current_ref < 0.)
          {  // Last recorded velocity and wished velocity (reference command) have different
             // signs
            // So we are asked to move in the opposite direction of the limit that was exceeded.
            // So we perform the movement as asked.
            // TODO: check if we cannot have a fast one instruction that check that signs are
            // different instead of multiplying
            command_interfaces_[0].set_value(current_ref);
          }
          else
          {
            // Either:
            // last recorded velocity and wished velocity (reference command) have the same sign
            // So we are asked to continue a movement that has already exceeded the limit.
            // And we stop the movement.
            // Or:
            // there is no recorded velocity sign so we don't know where we came from and we
            // don't move.
            command_interfaces_[0].set_value(0.);
          }
        }
        else
        {
          // switch right is the only active one
          if (limit_switch_right_violating_velocity_sign_ * current_ref < 0.)
          {  // Check if we cannot have a fast one instruction that check that signs are
             // different instead of multiplying
            command_interfaces_[0].set_value(current_ref);
          }
          else
          {
            // Either:
            // last recorded velocity and wished velocity (reference command) have the same sign
            // So we are asked to continue a movement that has already exceeded the limit.
            // And we stop the movement.
            // Or:
            // there is no recorded velocity sign so we don't know where we came from and we
            // don't move.
            command_interfaces_[0].set_value(0.);
          }
        }
      }
    }
    else
    {
      // Limit switches are inactive, set velocity to velocity reference command
      command_interfaces_[0].set_value(current_ref);
      record_limit_switch_state(limit_left_active, limit_right_active);
      return controller_interface::return_type::OK;
    }
  }
  else
  {
    // Velocity reference command is NaN, set velocity to 0.0
    command_interfaces_[0].set_value(0.);
    record_limit_switch_state(limit_left_active, limit_right_active);
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
