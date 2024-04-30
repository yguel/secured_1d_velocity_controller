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

#ifndef SECURED_1D_VELOCITY_CONTROLLER__SECURED_1D_VELOCITY_CONTROLLER_HPP_
#define SECURED_1D_VELOCITY_CONTROLLER__SECURED_1D_VELOCITY_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "secured_1d_control_interfaces/msg/secured_single_dof_state.hpp"
#include "secured_1d_velocity_controller/visibility_control.h"
#include "secured_1d_velocity_controller_parameters.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace secured_1d_velocity_controller
{

// Number of state interfaces
static constexpr size_t STATE_INTERFACES = 3;

// name constants for state interfaces
static constexpr size_t STATE_V_ITFS = 0;
static constexpr size_t STATE_START_LIMIT_ITFS = 1;
static constexpr size_t STATE_END_LIMIT_ITFS = 2;

// name constants for command interfaces
static constexpr size_t CMD_V_ITFS = 0;

// Velocity Sign Security Specification Mode
enum class control_mode_type : std::uint8_t
{
  UNKNOWN = 0,
  SECURE = 1,
  INSECURE = 2,
  SECURE_AND_LOG = 3,
  INSECURE_AND_LOG = 4
};

class Secured1dVelocityController : public controller_interface::ControllerInterface
{
public:
  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  Secured1dVelocityController();

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Command reference message type definition
  using ControllerReferenceMsg = std_msgs::msg::Float64;
  // Service message type definition
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  // State message type definition
  using ControllerStateMsg = secured_1d_control_interfaces::msg::SecuredSingleDOFState;

protected:
  std::shared_ptr<secured_1d_velocity_controller::ParamListener> param_listener_;
  secured_1d_velocity_controller::Params params_;

  std::string secured_joint_;
  double start_active_value_ = std::numeric_limits<double>::quiet_NaN();
  double end_active_value_ = std::numeric_limits<double>::quiet_NaN();
  double zero_velocity_tolerance_ = std::numeric_limits<double>::quiet_NaN();

  // Reference command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  // Services responsible for the setup of the control mode
  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_secure_mode_service_;
  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_log_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  // Controller State publisher
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

protected:
  inline void modify_secure_mode(const bool set_secure_mode)
  {
    control_mode_type mode = control_mode_.readFromNonRT();
    if (set_secure_mode)
    {
      if (mode == control_mode_type::INSECURE)
      {
        mode = control_mode_type::SECURE;
        control_mode_.writeFromNonRT(mode);
      }
      else if (mode == control_mode_type::INSECURE_AND_LOG)
      {
        mode = control_mode_type::SECURE_AND_LOG;
        control_mode_.writeFromNonRT(mode);
      }
    }
    else
    {
      if (mode == control_mode_type::SECURE)
      {
        mode = control_mode_type::INSECURE;
        control_mode_.writeFromNonRT(mode);
      }
      else if (mode == control_mode_type::SECURE_AND_LOG)
      {
        mode = control_mode_type::INSECURE_AND_LOG;
        control_mode_.writeFromNonRT(mode);
      }
    }
  }

  inline void modify_log_mode(const bool set_log_mode)
  {
    control_mode_type mode = control_mode_.readFromNonRT();
    if (set_log_mode)
    {
      if (mode == control_mode_type::SECURE)
      {
        mode = control_mode_type::SECURE_AND_LOG;
        control_mode_.writeFromNonRT(mode);
      }
      else if (mode == control_mode_type::INSECURE)
      {
        mode = control_mode_type::INSECURE_AND_LOG;
        control_mode_.writeFromNonRT(mode);
      }
    }
    else
    {
      if (mode == control_mode_type::SECURE_AND_LOG)
      {
        mode = control_mode_type::SECURE;
        control_mode_.writeFromNonRT(mode);
      }
      else if (mode == control_mode_type::INSECURE_AND_LOG)
      {
        mode = control_mode_type::INSECURE;
        control_mode_.writeFromNonRT(mode);
      }
    }
  }

protected:
  inline double admissible_velocity(
    bool start_limit_active, bool end_limit_active, const double reference_velocity) const
  {
    if (start_limit_active && reference_velocity < 0.0)
    {
      return 0.0;
    }
    if (end_limit_active && reference_velocity > 0.0)
    {
      return 0.0;
    }
    return reference_velocity;
  }
  inline double admissible_velocity_log(
    bool start_limit_active, bool end_limit_active, const double reference_velocity,
    bool & blocked) const
  {
    if (start_limit_active && reference_velocity < 0.0)
    {
      blocked = true;
      return 0.0;
    }
    if (end_limit_active && reference_velocity > 0.0)
    {
      blocked = true;
      return 0.0;
    }
    blocked = false;
    return reference_velocity;
  }
  bool velocity_reference_cmd_is_blocked_ = false;

protected:
  // Log variables
  bool start_limit_switch_prev_active_state_ = false;
  bool end_limit_switch_prev_active_state_ = false;
  std::string log_prefix_;

protected:
  // update method pointer
  controller_interface::return_type (Secured1dVelocityController::*update_method_ptr_)(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);

  controller_interface::return_type (Secured1dVelocityController::*update_method_ptr2_)(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);

  // update methods
  controller_interface::return_type update_secure(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);
  controller_interface::return_type update_insecure(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);
  controller_interface::return_type update_secure_and_log(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);
  controller_interface::return_type update_insecure_and_log(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);

  controller_interface::return_type update_with_state_published(
    const double reference_velocity, const double current_velocity, const bool start_limit_active,
    const bool end_limit_active, const rclcpp::Time & time, const rclcpp::Duration & period);

  // Log methods
  void record_limit_switch_state(bool limit_left_active, bool limit_right_active);
  void log_blocked_velocity_command(
    const bool block, const double current_ref, const double current_velocity);

protected:
  void set_control_mode(const control_mode_type mode);
  bool secured_mode_ = true;
  bool log_mode_ = false;

private:
  // callback for topic interface
  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace secured_1d_velocity_controller

#endif  // SECURED_1D_VELOCITY_CONTROLLER__SECURED_1D_VELOCITY_CONTROLLER_HPP_
