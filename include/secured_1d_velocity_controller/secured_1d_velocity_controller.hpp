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

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "secured_1d_velocity_controller/visibility_control.h"
#include "secured_1d_velocity_controller_parameters.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace secured_1d_velocity_controller
{

// Velocity Sign Security Specification Mode
enum class control_mode_type : std::uint8_t
{
  UNKNWON = 0,
  LIMITS_DISCOVERY = 1,
  LIMITS_DISCOVERED = 2,
  CALIBRATED = 3
};

/**
 * LimitDiscoveryState is a structure that holds the state of the
 * limit discovery process.
 */
struct LimitDiscoveryState
{
  bool start_discovered = false;
  std::size_t start_idx = 666666;
  bool end_discovered = false;
  std::size_t end_idx = 666666;

public:
  inline bool discovery_is_done() const { return start_discovered && end_discovered; }
  inline bool limit_known(const std::size_t limit_index) const
  {
    if (start_discovered && limit_index == start_idx)
    {
      return true;
    }
    if (end_discovered && limit_index == end_idx)
    {
      return true;
    }
    return false;
  }
};

/**
 * LimitMapping stores the respective position of the limit switches
 * with respect to the joint displacement direction.
 * The start limit is the limit switch that is reached when the joint
 * moves in the negative direction (backward).
 * The end limit is the limit switch that is reached when the joint
 * moves in the positive direction (forward).
 */
struct LimitMapping
{
public:
  /* Map interface indices to limit positions */
  std::size_t start_siidx;  //< start limit state_interface index;
  std::size_t end_siidx;    //< end limit state_interface index;

public:
  /* Map interface names to limit positions */
  std::string joint_name;
  std::string start_limit;
  std::string end_limit;

public:
  inline double admissible_velocity(
    const std::size_t limit_switch, const double reference_velocity, bool & blocked) const
  {
    if (limit_switch == start_siidx)
    {
      if (reference_velocity < 0.0)
      {
        blocked = true;
        return 0.0;
      }
    }
    else if (limit_switch == end_siidx)
    {
      if (reference_velocity > 0.0)
      {
        blocked = true;
        return 0.0;
      }
    }
    blocked = false;
    return reference_velocity;
  }

  inline double admissible_velocity(
    const std::size_t limit_switch, const double reference_velocity) const
  {
    if (limit_switch == start_siidx)
    {
      if (reference_velocity < 0.0)
      {
        return 0.0;
      }
    }
    else if (limit_switch == end_siidx)
    {
      if (reference_velocity > 0.0)
      {
        return 0.0;
      }
    }
    return reference_velocity;
  }

public:
  inline LimitMapping() = default;

  inline LimitMapping(
    const std::string & joint_name, const std::string & start_limit, const std::string & end_limit,
    const std::size_t start_siidx, const std::size_t end_siidx)
  : start_siidx(start_siidx),
    end_siidx(end_siidx),
    joint_name(joint_name),
    start_limit(start_limit),
    end_limit(end_limit)
  {
  }

  inline LimitMapping(const LimitMapping & other)
  : start_siidx(other.start_siidx),
    end_siidx(other.end_siidx),
    joint_name(other.joint_name),
    start_limit(other.start_limit),
    end_limit(other.end_limit)
  {
  }

  inline LimitMapping & operator=(const LimitMapping & other)
  {
    start_siidx = other.start_siidx;
    end_siidx = other.end_siidx;
    joint_name = other.joint_name;
    start_limit = other.start_limit;
    end_limit = other.end_limit;
    return *this;
  }

  inline bool operator==(const LimitMapping & other) const
  {
    return start_siidx == other.start_siidx && end_siidx == other.end_siidx &&
           joint_name == other.joint_name && start_limit == other.start_limit &&
           end_limit == other.end_limit;
  }

  inline bool operator!=(const LimitMapping & other) const { return !(*this == other); }
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
  // State message type definition
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<secured_1d_velocity_controller::ParamListener> param_listener_;
  secured_1d_velocity_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Reference command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  /**
  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_velocity_sign_security_mode_service_;
  */
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

protected:
  LimitDiscoveryState discovery_state_;
  LimitMapping limit_map_;
  size_t limit_switch0_idx_ = 666666;
  size_t limit_switch1_idx_ = 666666;

  /** Limit discovery mode methods */
  double discover_limits__limit_reached_command(
    const std::size_t limit_index, const double current_velocity, const double reference_velocity,
    bool & blocked);

  // Record velocity sign associated with limit violated
  double zero_velocity_tolerance_ = 1.e-3;
  double limit_switch_left_violating_velocity_sign_ = 0.0;
  double limit_switch_right_violating_velocity_sign_ = 0.0;
  bool limit_switch_left_prev_active_state_ = false;
  bool limit_switch_right_prev_active_state_ = false;
  bool velocity_reference_cmd_is_blocked_ = false;

private:
  // callback for topic interface
  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  void record_limit_switch_state(bool limit_left_active, bool limit_right_active);
  void log_blocked_velocity_command(
    const bool block, const double current_ref, const double current_velocity);
};

}  // namespace secured_1d_velocity_controller

#endif  // SECURED_1D_VELOCITY_CONTROLLER__SECURED_1D_VELOCITY_CONTROLLER_HPP_
