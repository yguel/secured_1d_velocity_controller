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

#ifndef TEST_SECURED_1D_VELOCITY_CONTROLLER_HPP_
#define TEST_SECURED_1D_VELOCITY_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "secured_1d_velocity_controller/secured_1d_velocity_controller.hpp"

using ControllerStateMsg =
  secured_1d_velocity_controller::Secured1dVelocityController::ControllerStateMsg;
using ControllerReferenceMsg =
  secured_1d_velocity_controller::Secured1dVelocityController::ControllerReferenceMsg;
using ControllerModeSrvType =
  secured_1d_velocity_controller::Secured1dVelocityController::ControllerModeSrvType;
using control_mode_type = secured_1d_velocity_controller::control_mode_type;
using secured_1d_velocity_controller::CMD_V_ITFS;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableSecured1dVelocityController
: public secured_1d_velocity_controller::Secured1dVelocityController
{
  FRIEND_TEST(Secured1dVelocityControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(Secured1dVelocityControllerTest, activate_success);
  FRIEND_TEST(Secured1dVelocityControllerTest, reactivate_success);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret =
      secured_1d_velocity_controller::Secured1dVelocityController::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS)
    {
      ref_subscriber_wait_set_.add_subscription(ref_subscriber_);
    }
    return ret;
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerReferenceMsg msg was received, false if timeout.
   */
  bool wait_for_command(
    rclcpp::Executor & executor, rclcpp::WaitSet & subscriber_wait_set,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, ref_subscriber_wait_set_, timeout);
  }

private:
  rclcpp::WaitSet ref_subscriber_wait_set_;
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class Secured1dVelocityControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    std::string base_name = "/test_secured_1d_velocity_controller/";

    std::string ref_topic = base_name + reference_topic_;
    command_publisher_node_ = std::make_shared<rclcpp::Node>("reference_command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      ref_topic, rclcpp::SystemDefaultsQoS());

    std::string security_service = base_name + security_service_name_;
    security_service_caller_node_ = std::make_shared<rclcpp::Node>("security_service_caller");
    security_service_client_ =
      security_service_caller_node_->create_client<ControllerModeSrvType>(security_service);

    std::string log_service = base_name + log_service_name_;
    log_service_caller_node_ = std::make_shared<rclcpp::Node>("security_service_caller");
    log_service_client_ =
      log_service_caller_node_->create_client<ControllerModeSrvType>(log_service);
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_secured_1d_velocity_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    // Setup command interfaces
    //=========================
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(reference_command_values_.size());
    command_ifs.reserve(reference_command_values_.size());

    {  // One joint, one command interface
      // Set one valid command interface value
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        joint_name_, hardware_interface::HW_IF_VELOCITY, &reference_command_values_[CMD_V_ITFS]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    // Setup state interfaces
    //=======================
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(state_values_.size());
    state_ifs.reserve(state_values_.size());

    for (size_t i = 0; i < state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(hardware_interface::StateInterface(
        state_base_names_[i], state_interface_names_[i], &state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    // Copy the interfaces to the controller (commands and states)
    //============================================================
    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_secured_1d_velocity_controller/state", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands(const double & velocity = 0.0)
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.data = velocity;

    command_publisher_->publish(msg);
  }

  std::shared_ptr<ControllerModeSrvType::Response> call_security_service(
    const bool set_secure, rclcpp::Executor & executor)
  {
    auto request = std::make_shared<ControllerModeSrvType::Request>();
    request->data = set_secure;

    bool wait_for_service_ret =
      security_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Security mode service is not available!");
    }
    auto result = security_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }

  std::shared_ptr<ControllerModeSrvType::Response> call_log_service(
    const bool set_log, rclcpp::Executor & executor)
  {
    auto request = std::make_shared<ControllerModeSrvType::Request>();
    request->data = set_log;

    bool wait_for_service_ret =
      log_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Log mode service is not available!");
    }
    auto result = log_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }

protected:
  // Controller-related parameters
  std::string joint_name_ = "joint1";
  std::vector<std::string> state_base_names_ = {
    "joint1", "negative_limit_sensor", "positive_limit_sensor"};
  std::vector<std::string> state_interface_names_ = {"velocity", "switch", "switch"};
  double start_limit_active_value_ = 1.0;
  double end_limit_active_value_ = 0.0;
  double zero_velocity_tolerance_ = 0.0002;
  double reset_velocity_ = 0.0;
  std::string reference_topic_ = "velocity_command";
  std::string security_service_default_mode_ = "SECURE";
  std::string security_service_name_ = "set_security_mode";
  std::string log_service_default_mode_ = "LOG";
  std::string log_service_name_ = "set_logging_mode";

  std::array<double, 3> state_values_ = {0., 0., 0.};         // velocity, start_limit, end_limit
  std::array<double, 3> state_values_ok_ = {2.5, 0., 0.};     // velocity, start_limit, end_limit
  std::array<double, 3> state_values_start_ = {1.5, 1., 0.};  // velocity, start_limit, end_limit
  std::array<double, 3> state_values_end_ = {-1.5, 0., 1.};   // velocity, start_limit, end_limit
  std::array<double, 3> state_values_both_ = {3.5, 1., 1.};   // velocity, start_limit,
  std::array<double, 1> reference_command_values_ = {1.};
  std::array<double, 1> reference_value_pos_ = {1.8};
  std::array<double, 1> reference_value_neg_ = {-1.8};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableSecured1dVelocityController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr security_service_caller_node_;
  rclcpp::Client<ControllerModeSrvType>::SharedPtr security_service_client_;
  rclcpp::Node::SharedPtr log_service_caller_node_;
  rclcpp::Client<ControllerModeSrvType>::SharedPtr log_service_client_;
};

#endif  // TEST_SECURED_1D_VELOCITY_CONTROLLER_HPP_
