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

#ifndef SECURED_1D_VELOCITY_CONTROLLER__VALIDATE_SECURED_1D_VELOCITY_CONTROLLER_PARAMETERS_HPP_
#define SECURED_1D_VELOCITY_CONTROLLER__VALIDATE_SECURED_1D_VELOCITY_CONTROLLER_PARAMETERS_HPP_

#include <fmt/core.h>
#include <string>

#include <tl_expected/expected.hpp>

#include <rclcpp/rclcpp.hpp>

namespace secured_1d_velocity_controller
{

tl::expected<void, std::string> is_finite(rclcpp::Parameter const & parameter)
{
  double param_value = parameter.as_double();
  if (!std::isfinite(param_value))
  {
    return tl::make_unexpected(fmt::format(
      "Invalid value {} for parameter {}. Expected a finite value.", param_value,
      parameter.get_name()));
  }
  return {};  // OK
}

}  // namespace secured_1d_velocity_controller

#endif  // SECURED_1D_VELOCITY_CONTROLLER__VALIDATE_SECURED_1D_VELOCITY_CONTROLLER_PARAMETERS_HPP_
