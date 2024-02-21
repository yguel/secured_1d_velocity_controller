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
  INFERRED_VELOCITY_SIGN_SECURITY = 1,
  SPECIFIED_VELOCITY_SIGN_SECURITY = 2
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

  // Record velocity sign associated with limit violated
  double limit_switch_left_violating_velocity_sign_ = 0.0;
  double limit_switch_right_violating_velocity_sign_ = 0.0;
  bool limit_switch_left_prev_active_state_ = false;
  bool limit_switch_right_prev_active_state_ = false;

private:
  // callback for topic interface
  SECURED_1D_VELOCITY_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void record_limit_switch_state(bool limit_left_active, bool limit_right_active);
};

}  // namespace secured_1d_velocity_controller

#endif  // SECURED_1D_VELOCITY_CONTROLLER__SECURED_1D_VELOCITY_CONTROLLER_HPP_
