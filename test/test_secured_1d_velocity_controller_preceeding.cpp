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

  // Check parameters
  ASSERT_EQ(controller_->params_.joint, std::string(""));
  ASSERT_EQ(controller_->params_.start_limit.state_interface, std::string(""));
  ASSERT_EQ(controller_->params_.end_limit.state_interface, std::string(""));
  ASSERT_EQ(controller_->params_.reference_topic, std::string(""));
  ASSERT_EQ(controller_->params_.security_mode_service.default_mode, std::string(""));
  ASSERT_EQ(controller_->params_.security_mode_service.service, std::string(""));
  ASSERT_EQ(controller_->params_.log_mode_service.default_mode, std::string(""));
  ASSERT_EQ(controller_->params_.log_mode_service.service, std::string(""));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check parameters
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

  ASSERT_EQ(
    controller_->params_.security_mode_service.default_mode, security_service_default_mode_);
  ASSERT_EQ(controller_->params_.security_mode_service.service, security_service_name_);

  ASSERT_EQ(controller_->params_.log_mode_service.default_mode, log_service_default_mode_);
  ASSERT_EQ(controller_->params_.log_mode_service.service, log_service_name_);
}

TEST_F(Secured1dVelocityControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), reference_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_base_names_[i] + "/" + state_interface_names_[i]);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
