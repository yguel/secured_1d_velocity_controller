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
  control_mode_.initRT(control_mode_type::SECURE);

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

control_mode_type Secured1dVelocityController::define_control_mode(
  const bool security_enabled, const bool log_enabled) const
{
  if (security_enabled)
  {
    if (log_enabled)
    {
      return control_mode_type::SECURE_AND_LOG;
    }
    else
    {
      return control_mode_type::SECURE;
    }
  }
  else
  {
    if (log_enabled)
    {
      return control_mode_type::INSECURE_AND_LOG;
    }
    else
    {
      return control_mode_type::INSECURE;
    }
  }
}

void Secured1dVelocityController::set_control_mode(const control_mode_type mode)
{
  switch (mode)
  {
    case control_mode_type::SECURE:
    {
      if (params_.publish_state)
      {
        update_method_ptr_ = &Secured1dVelocityController::update_with_state_published;
        update_method_ptr2_ = &Secured1dVelocityController::update_secure;
      }
      else
      {
        update_method_ptr_ = &Secured1dVelocityController::update_secure;
        update_method_ptr2_ = nullptr;
      }
      log_prefix_ = std::string("");
      secured_mode_ = true;
      log_mode_ = false;
      break;
    }
    case control_mode_type::INSECURE:
    {
      if (params_.publish_state)
      {
        update_method_ptr_ = &Secured1dVelocityController::update_with_state_published;
        update_method_ptr2_ = &Secured1dVelocityController::update_insecure;
      }
      else
      {
        update_method_ptr_ = &Secured1dVelocityController::update_insecure;
        update_method_ptr2_ = nullptr;
      }
      log_prefix_ = std::string("");
      secured_mode_ = false;
      log_mode_ = false;
      break;
    }
    case control_mode_type::SECURE_AND_LOG:
    {
      if (params_.publish_state)
      {
        update_method_ptr_ = &Secured1dVelocityController::update_with_state_published;
        update_method_ptr2_ = &Secured1dVelocityController::update_secure_and_log;
      }
      else
      {
        update_method_ptr_ = &Secured1dVelocityController::update_secure_and_log;
        update_method_ptr2_ = nullptr;
      }
      log_prefix_ = std::string("");
      secured_mode_ = true;
      log_mode_ = true;
      break;
    }
    case control_mode_type::INSECURE_AND_LOG:
    {
      if (params_.publish_state)
      {
        update_method_ptr_ = &Secured1dVelocityController::update_with_state_published;
        update_method_ptr2_ = &Secured1dVelocityController::update_insecure_and_log;
      }
      else
      {
        update_method_ptr_ = &Secured1dVelocityController::update_insecure_and_log;
        update_method_ptr2_ = nullptr;
      }
      log_prefix_ = std::string("Insecure mode, should have ");
      secured_mode_ = false;
      log_mode_ = true;
      break;
    }
    default:
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unknown control mode");
      update_method_ptr_ = &Secured1dVelocityController::update_secure;
      log_prefix_ = std::string("");
      secured_mode_ = true;
      log_mode_ = false;
      break;
    }
  }

  control_mode_.writeFromNonRT(mode);
}

void Secured1dVelocityController::set_publish_state(bool publish_state)
{
  params_.publish_state = publish_state;
  set_control_mode(*(control_mode_.readFromNonRT()));
}

bool Secured1dVelocityController::setup_state_publisher()
{
  if (params_.publish_state)
  {
    try
    {
      // State publisher
      s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
        std::string("~/") + STATE_TOPIC_NAME, rclcpp::SystemDefaultsQoS());
      state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
    }
    catch (const std::exception & e)
    {
      fprintf(
        stderr,
        "Exception thrown during publisher creation at configure stage with message : %s \n",
        e.what());
      return false;
    }

    state_publisher_->lock();
    state_publisher_->msg_.dof_state.header.frame_id = params_.joint;
    state_publisher_->unlock();

    // Set the update method to the one that publishes the state
    set_control_mode(*(control_mode_.readFromNonRT()));
    return true;
  }
  return true;
}

controller_interface::CallbackReturn Secured1dVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  secured_joint_ = params_.joint;
  start_active_value_ = params_.start_limit.active_value;
  end_active_value_ = params_.end_limit.active_value;
  zero_velocity_tolerance_ = params_.zero_velocity_tolerance;

  /** Check if the start and end limit switches are the same
   * If they are the same this is a configuration error
   */
  if (params_.start_limit.state_interface == params_.end_limit.state_interface)
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "The state interface for start and end limit switches cannot be the same. Please, check the "
      "configuration.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Define the services callbacks
  auto set_secure_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    modify_secure_mode(request->data);
    response->success = true;
  };

  auto set_log_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    modify_log_mode(request->data);
    response->success = true;
  };

  /* Define the service responsible to set in secure mode */
  std::string security_mode_service_name =
    std::string("~/") + params_.security_mode_service.service;
  set_secure_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    security_mode_service_name, set_secure_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  /* Define the service responsible to set in log mode */
  std::string log_mode_service_name = std::string("~/") + params_.log_mode_service.service;
  set_log_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    log_mode_service_name, set_log_mode_service_callback, rmw_qos_profile_services_hist_keep_all);

  // Set the control mode
  bool security_enabled = true;
  if (std::string("INSECURE") == params_.security_mode_service.default_mode)
  {
    security_enabled = false;
  }
  bool log_enabled = false;
  if (std::string("LOG") == params_.log_mode_service.default_mode)
  {
    log_enabled = true;
  }
  control_mode_type mode = define_control_mode(security_enabled, log_enabled);
  control_mode_.writeFromNonRT(mode);
  set_control_mode(mode);

  // State publisher
  if (!setup_state_publisher())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Command Velocity Subscriber
  const std::string ref_topic = std::string("~/") + params_.reference_topic;
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    ref_topic, subscribers_qos,
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
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration
Secured1dVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(1);
  command_interfaces_config.names.push_back(
    secured_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
Secured1dVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  /** Motor states */
  state_interfaces_config.names.resize(STATE_INTERFACES, "");
  state_interfaces_config.names[STATE_V_ITFS] =
    secured_joint_ + std::string("/") + hardware_interface::HW_IF_VELOCITY;

  /** Limit switch states*/
  state_interfaces_config.names[STATE_START_LIMIT_ITFS] = params_.start_limit.state_interface;
  state_interfaces_config.names[STATE_END_LIMIT_ITFS] = params_.end_limit.state_interface;

  return state_interfaces_config;
}

controller_interface::CallbackReturn Secured1dVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)());

  // Set default value for command interface
  command_interfaces_[CMD_V_ITFS].set_value(0.0);

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
    start_limit_switch_prev_active_state_ != limit_left_active ||
    end_limit_switch_prev_active_state_ != limit_right_active)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Limit switch state changed: start: (%d->%d), end: (%d->%d)",
      start_limit_switch_prev_active_state_, limit_left_active, end_limit_switch_prev_active_state_,
      limit_right_active);
  }
  start_limit_switch_prev_active_state_ = limit_left_active;
  end_limit_switch_prev_active_state_ = limit_right_active;
}

inline void Secured1dVelocityController::log_blocked_velocity_command(
  const bool block, const double current_ref, const double current_velocity)
{
  auto current_vel_mag = std::abs(current_velocity);
  if (current_vel_mag < zero_velocity_tolerance_)
  {
    return;
  }

  std::string msg;

  if (block && !velocity_reference_cmd_is_blocked_)
  {
    msg =
      log_prefix_ +
      std::string("BLOCKED velocity command %f because of limit switch (current_velocity was %f)");
    velocity_reference_cmd_is_blocked_ = true;
  }
  else
  {
    if (!block && velocity_reference_cmd_is_blocked_)
    {
      msg = log_prefix_ + std::string("UNBLOCKED velocity command %f (current_velocity was %f)");
      velocity_reference_cmd_is_blocked_ = false;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), msg.c_str(), current_ref, current_velocity);
}

controller_interface::return_type Secured1dVelocityController::update_insecure(
  const double reference_velocity, const double /*current_velocity*/,
  const bool /*start_limit_active*/, const bool /*end_limit_active*/, const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  command_interfaces_[CMD_V_ITFS].set_value(reference_velocity);
  return controller_interface::return_type::OK;
}

controller_interface::return_type Secured1dVelocityController::update_secure(
  const double reference_velocity, const double /*current_velocity*/, const bool start_limit_active,
  const bool end_limit_active, const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!std::isnan(reference_velocity))
  {  // Velocity reference command is well defined
    if (start_limit_active && end_limit_active)
    {
      // Both limit switches are active (that is very bad!), set velocity to 0.0
      RCLCPP_ERROR(get_node()->get_logger(), "Both limit switches are active");
      command_interfaces_[CMD_V_ITFS].set_value(0.);
      return controller_interface::return_type::OK;
    }

    const double vel_cmd =
      admissible_velocity(start_limit_active, end_limit_active, reference_velocity);

    command_interfaces_[CMD_V_ITFS].set_value(vel_cmd);
    return controller_interface::return_type::OK;
  }
  else
  {
    // Velocity reference command is NaN, set velocity to 0.0
    command_interfaces_[CMD_V_ITFS].set_value(0.);
    return controller_interface::return_type::OK;
  }
}

controller_interface::return_type Secured1dVelocityController::update_secure_and_log(
  const double reference_velocity, const double current_velocity, const bool start_limit_active,
  const bool end_limit_active, const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!std::isnan(reference_velocity))
  {  // Velocity reference command is well defined
    if (start_limit_active && end_limit_active)
    {
      // Both limit switches are active (that is very bad!), set velocity to 0.0
      RCLCPP_ERROR(get_node()->get_logger(), "Both limit switches are active");
      command_interfaces_[CMD_V_ITFS].set_value(0.);
      record_limit_switch_state(start_limit_active, end_limit_active);
      return controller_interface::return_type::OK;
    }

    bool blocked = false;
    double vel_cmd =
      admissible_velocity_log(start_limit_active, end_limit_active, reference_velocity, blocked);

    command_interfaces_[CMD_V_ITFS].set_value(vel_cmd);
    record_limit_switch_state(start_limit_active, end_limit_active);
    // Intelligent log the blocked state of the velocity
    log_blocked_velocity_command(blocked, reference_velocity, current_velocity);
    return controller_interface::return_type::OK;
  }
  else
  {
    // Velocity reference command is NaN, set velocity to 0.0
    command_interfaces_[CMD_V_ITFS].set_value(0.);
    record_limit_switch_state(start_limit_active, end_limit_active);
    // Log the fact that we blocked the velocity
    log_blocked_velocity_command(true, reference_velocity, current_velocity);
    return controller_interface::return_type::OK;
  }
}

controller_interface::return_type Secured1dVelocityController::update_insecure_and_log(
  const double reference_velocity, const double current_velocity, const bool start_limit_active,
  const bool end_limit_active, const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool blocked = false;

  if (start_limit_active && end_limit_active)
  {
    // Both limit switches are active (that is very bad!), set velocity to 0.0
    RCLCPP_WARN(get_node()->get_logger(), "Both limit switches are active");
    record_limit_switch_state(start_limit_active, end_limit_active);
    blocked = true;
  }
  else
  {
    admissible_velocity_log(start_limit_active, end_limit_active, reference_velocity, blocked);
    record_limit_switch_state(start_limit_active, end_limit_active);
  }

  // Intelligent log the blocked state of the velocity
  log_blocked_velocity_command(blocked, reference_velocity, current_velocity);

  command_interfaces_[CMD_V_ITFS].set_value(reference_velocity);
  return controller_interface::return_type::OK;
}

controller_interface::return_type Secured1dVelocityController::update_with_state_published(
  const double reference_velocity, const double current_velocity, const bool start_limit_active,
  const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period)
{
#define METHOD_POINTER_CALL(ptrToMethod) (this->*(ptrToMethod))
  controller_interface::return_type ret = METHOD_POINTER_CALL(update_method_ptr2_)(
    reference_velocity, current_velocity, start_limit_active, end_limit_active, time, period);
#undef METHOD_POINTER_CALL

  double computed_ref = command_interfaces_[CMD_V_ITFS].get_value();

  bool security_triggered = false;
  // Check if the computed reference is different from the reference velocity
  // If it is different, then the security has been triggered because otherwise
  // the controller just forwards the reference velocity
  if (computed_ref != reference_velocity)
  {
    security_triggered = true;
  }

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.dof_state.header.stamp = time;
  state_publisher_->msg_.dof_state.set_point = reference_velocity;
  state_publisher_->msg_.dof_state.process_value = current_velocity;
  state_publisher_->msg_.dof_state.error = current_velocity - reference_velocity;
  state_publisher_->msg_.dof_state.command = computed_ref;
  state_publisher_->msg_.secured_mode = secured_mode_;
  state_publisher_->msg_.security_triggered = security_triggered;
  state_publisher_->unlockAndPublish();

  return ret;
}

controller_interface::return_type Secured1dVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto current_ref_msg = input_ref_.readFromRT();

  if (!current_ref_msg)
  {  // ERROR if no velocity command received
    RCLCPP_ERROR(get_node()->get_logger(), "No velocity command received.");
    return controller_interface::return_type::ERROR;
  }

  const auto current_ref = (*current_ref_msg)->data;
  const auto current_velocity = state_interfaces_[STATE_V_ITFS].get_value();
  const auto current_start_limit_switch = state_interfaces_[STATE_START_LIMIT_ITFS].get_value();
  const auto current_end_limit_switch = state_interfaces_[STATE_END_LIMIT_ITFS].get_value();

  const bool start_limit_active = (current_start_limit_switch == start_active_value_);
  const bool end_limit_active = (current_end_limit_switch == end_active_value_);

#define METHOD_POINTER_CALL(ptrToMethod) (this->*(ptrToMethod))
  return METHOD_POINTER_CALL(update_method_ptr_)(
    current_ref, current_velocity, start_limit_active, end_limit_active, time, period);
#undef METHOD_POINTER_CALL
}

}  // namespace secured_1d_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  secured_1d_velocity_controller::Secured1dVelocityController,
  controller_interface::ControllerInterface)
