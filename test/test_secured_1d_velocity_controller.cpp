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

#include "test_secured_1d_velocity_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using secured_1d_velocity_controller::CMD_V_ITFS;
using secured_1d_velocity_controller::control_mode_type;
using secured_1d_velocity_controller::STATE_END_LIMIT_ITFS;
using secured_1d_velocity_controller::STATE_START_LIMIT_ITFS;
using secured_1d_velocity_controller::STATE_V_ITFS;

class Secured1dVelocityControllerTest
: public Secured1dVelocityControllerFixture<TestableSecured1dVelocityController>
{
};

TEST_F(Secured1dVelocityControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  /** Test that parameters are initialized with default values
   * ==========================================================
   */
  ASSERT_EQ(controller_->params_.joint, std::string("joint_name"));

  ASSERT_EQ(
    controller_->params_.start_limit.state_interface,
    std::string("start_limit_sensor/limit_switch"));
  ASSERT_EQ(controller_->params_.start_limit.active_value, static_cast<double>(1.0));

  ASSERT_EQ(
    controller_->params_.end_limit.state_interface, std::string("end_limit_sensor/limit_switch"));
  ASSERT_EQ(controller_->params_.end_limit.active_value, static_cast<double>(1.0));

  ASSERT_EQ(controller_->params_.reference_topic, std::string("reference_velocity"));

  ASSERT_EQ(controller_->params_.zero_velocity_tolerance, static_cast<double>(1e-6));

  ASSERT_EQ(controller_->params_.security_mode_service.default_mode, std::string("SECURE"));
  ASSERT_EQ(controller_->params_.security_mode_service.service, std::string("set_secure_mode"));
  ASSERT_EQ(controller_->params_.log_mode_service.default_mode, std::string("NO_LOG"));
  ASSERT_EQ(controller_->params_.log_mode_service.service, std::string("set_log_mode"));

  /** Test that controller can correctly pass the configure stage
   * =============================================================
   */
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  /** Test that the parameters are correctly set
   * ============================================
   */
  ASSERT_EQ(controller_->params_.joint, joint_name_);

  ASSERT_EQ(
    controller_->params_.start_limit.state_interface,
    state_base_names_[STATE_START_LIMIT_ITFS] + std::string("/") +
      state_interface_names_[STATE_START_LIMIT_ITFS]);
  ASSERT_EQ(controller_->params_.start_limit.active_value, start_limit_active_value_);

  ASSERT_EQ(
    controller_->params_.end_limit.state_interface, state_base_names_[STATE_END_LIMIT_ITFS] +
                                                      std::string("/") +
                                                      state_interface_names_[STATE_END_LIMIT_ITFS]);
  ASSERT_EQ(controller_->params_.end_limit.active_value, end_limit_active_value_);

  ASSERT_EQ(controller_->params_.reference_topic, reference_topic_);

  ASSERT_EQ(controller_->params_.zero_velocity_tolerance, zero_velocity_tolerance_);

  ASSERT_EQ(
    controller_->params_.security_mode_service.default_mode, security_service_default_mode_);
  ASSERT_EQ(controller_->params_.security_mode_service.service, security_service_name_);

  ASSERT_EQ(controller_->params_.log_mode_service.default_mode, log_service_default_mode_);
  ASSERT_EQ(controller_->params_.log_mode_service.service, log_service_name_);
}

TEST_F(Secured1dVelocityControllerTest, check_exported_intefaces)
{
  SetUpController();

  /** Test that controller can correctly pass the configure stage
   * =============================================================
   */
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  /** Commands
   * =========
   */
  auto command_intefaces = controller_->command_interface_configuration();
  /** Test that the number of command interfaces is correct*/
  ASSERT_EQ(command_intefaces.names.size(), reference_command_values_.size());
  /** Test that the order and the name of each interface is correct */
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  /** States
   * =======
   */
  auto state_intefaces = controller_->state_interface_configuration();
  /** Test that the number of state interfaces is correct*/
  ASSERT_EQ(state_intefaces.names.size(), state_values_.size());
  /** Test that the order and the name of each interface is correct */
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_base_names_[i] + "/" + state_interface_names_[i]);
  }
}

TEST_F(Secured1dVelocityControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*msg)->data, reset_velocity_);
}

TEST_F(Secured1dVelocityControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(Secured1dVelocityControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(Secured1dVelocityControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    controller_->command_interfaces_[CMD_V_ITFS].get_value(),
    reference_command_values_[CMD_V_ITFS]);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CMD_V_ITFS].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), reset_velocity_);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

double another_value(double value)
{
  if (value == 0.0)
  {
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}

TEST_F(Secured1dVelocityControllerTest, update_logic_secure_mode)
{
  SetUpController("test_secured_1d_velocity_controller", false /*publish_state*/);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->set_control_mode(control_mode_type::SECURE);

  std::vector<double> state_values = {0.0, 0.0, 0.0};
  std::vector<double> cmd_values = {0.0};

  double start_active_value = controller_->start_active_value_;
  double start_inactive_value = another_value(start_active_value);
  double end_active_value = controller_->end_active_value_;
  double end_inactive_value = another_value(end_active_value);

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  //==================================================
  // 1. Test positive velocity and no limit activated
  //==================================================

  // set command statically
  static constexpr double TEST_VELOCITY1 = 23.24;
  msg->data = TEST_VELOCITY1;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY1);

  //===================================================
  // 2. Test negative velocity and no limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY2 = -23.24;
  msg->data = TEST_VELOCITY2;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY2);

  //===================================================
  // 3. Test positive velocity and start limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY3 = 23.24;
  msg->data = TEST_VELOCITY3;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY3);

  //===================================================
  // 4. Test positive velocity and end limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY4 = 23.24;
  msg->data = TEST_VELOCITY4;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), 0.);

  //===================================================
  // 5. Test negative velocity and start limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY5 = -23.24;
  msg->data = TEST_VELOCITY5;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), 0.);

  //===================================================
  // 6. Test negative velocity and end limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY6 = -23.24;
  msg->data = TEST_VELOCITY6;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY6);

  //=====================================================
  // 7. Test positive velocity and both limits activated
  //=====================================================

  // set command statically
  static constexpr double TEST_VELOCITY7 = 23.24;
  msg->data = TEST_VELOCITY7;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), 0.);

  //=====================================================
  // 8. Test negative velocity and both limits activated
  //=====================================================

  // set command statically
  static constexpr double TEST_VELOCITY8 = -23.24;
  msg->data = TEST_VELOCITY8;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), 0.);
}

TEST_F(Secured1dVelocityControllerTest, update_logic_insecure_mode)
{
  SetUpController("test_secured_1d_velocity_controller", false /*publish_state*/);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->set_control_mode(control_mode_type::INSECURE);

  std::vector<double> state_values = {0.0, 0.0, 0.0};
  std::vector<double> cmd_values = {0.0};

  double start_active_value = controller_->start_active_value_;
  double start_inactive_value = another_value(start_active_value);
  double end_active_value = controller_->end_active_value_;
  double end_inactive_value = another_value(end_active_value);

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  //==================================================
  // 1. Test positive velocity and no limit activated
  //==================================================

  // set command statically
  static constexpr double TEST_VELOCITY1 = 23.24;
  msg->data = TEST_VELOCITY1;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY1);

  //===================================================
  // 2. Test negative velocity and no limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY2 = -23.24;
  msg->data = TEST_VELOCITY2;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY2);

  //===================================================
  // 3. Test positive velocity and start limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY3 = 23.24;
  msg->data = TEST_VELOCITY3;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY3);

  //===================================================
  // 4. Test positive velocity and end limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY4 = 23.24;
  msg->data = TEST_VELOCITY4;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY4);

  //===================================================
  // 5. Test negative velocity and start limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY5 = -23.24;
  msg->data = TEST_VELOCITY5;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY5);

  //===================================================
  // 6. Test negative velocity and end limit activated
  //===================================================

  // set command statically
  static constexpr double TEST_VELOCITY6 = -23.24;
  msg->data = TEST_VELOCITY6;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY6);

  //=====================================================
  // 7. Test positive velocity and both limits activated
  //=====================================================

  // set command statically
  static constexpr double TEST_VELOCITY7 = 23.24;
  msg->data = TEST_VELOCITY7;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY7);

  //=====================================================
  // 8. Test negative velocity and both limits activated
  //=====================================================

  // set command statically
  static constexpr double TEST_VELOCITY8 = -23.24;
  msg->data = TEST_VELOCITY8;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_active_value;
  state_values[STATE_END_LIMIT_ITFS] = end_active_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY8);
}

TEST_F(Secured1dVelocityControllerTest, test_setting_secure_mode_service)
{
  SetUpController("test_secured_1d_velocity_controller", false /*publish_state*/);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  // initially set to secure
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // should stay secure
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);

  // set to insecure
  ASSERT_NO_THROW(call_security_service(false, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::INSECURE);

  // set back to secure
  ASSERT_NO_THROW(call_security_service(true, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);
}

TEST_F(Secured1dVelocityControllerTest, test_setting_secure_mode_service_while_publishing_status)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  // initially set to secure
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // should stay secure
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);

  // set to insecure
  ASSERT_NO_THROW(call_security_service(false, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::INSECURE);

  // set back to secure
  ASSERT_NO_THROW(call_security_service(true, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), control_mode_type::SECURE);
}

TEST_F(Secured1dVelocityControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->set_control_mode(control_mode_type::SECURE);

  std::vector<double> state_values = {0.0, 0.0, 0.0};
  std::vector<double> cmd_values = {0.0};

  double start_active_value = controller_->start_active_value_;
  double start_inactive_value = another_value(start_active_value);
  double end_active_value = controller_->end_active_value_;
  double end_inactive_value = another_value(end_active_value);

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  ControllerStateMsg msg;

  //================================================================
  // 1. Test positive velocity and no limit activated (SECURED MODE)
  //================================================================

  // set command statically
  static constexpr double TEST_VELOCITY1 = 23.24;
  msg->data = TEST_VELOCITY1;
  controller_->input_ref_.writeFromNonRT(msg);

  // set state values for limits
  state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
  state_values[STATE_END_LIMIT_ITFS] = end_inactive_value;
  mock_states(state_values);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), TEST_VELOCITY1);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, TEST_VELOCITY1);
  ASSERT_EQ(msg.secured_mode, true);
  ASSERT_EQ(msg.security_enabled, false);

  /** TODO check
    //==================================================================
    // 4. Test positive velocity and end limit activated (SECURED MODE)
    //==================================================================

    // set command statically
    static constexpr double TEST_VELOCITY4 = 23.24;
    msg->data = TEST_VELOCITY4;
    controller_->input_ref_.writeFromNonRT(msg);

    // set state values for limits
    state_values[STATE_START_LIMIT_ITFS] = start_inactive_value;
    state_values[STATE_END_LIMIT_ITFS] = end_active_value;
    mock_states(state_values);

    ASSERT_EQ(
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    EXPECT_EQ(controller_->command_interfaces_[CMD_V_ITFS].get_value(), 0.);
    */
}

/**
TEST_F(Secured1dVelocityControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, reference_command_values_[CMD_V_ITFS]);

  publish_commands(reference_value_pos_[CMD_V_ITFS]);
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, reference_value_pos_[CMD_V_ITFS]);
}
*/

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
