// Copyright 2023 Kenji Brameld
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

#include "nao_loopback/nao_loopback.hpp"

#include <chrono>

#include "conversion.hpp"

#define FREQUENCY 82  // Hz
#define PERIOD std::chrono::microseconds(1000000 / FREQUENCY)

namespace nao_loopback
{

NaoLoopback::NaoLoopback(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_loopback", options}
{
  jointPositionsPub =
    create_publisher<nao_lola_sensor_msgs::msg::JointPositions>("sensors/joint_positions", 10);
  jointStatesPub =
    create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  jointPositionsSub =
    create_subscription<nao_lola_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_lola_command_msgs::msg::JointPositions::SharedPtr cmd) {
      if (cmd->indexes.size() != cmd->positions.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "Incorrect message received for nao_lola_command_msgs::msg::JointPositions. "
          "Angles and Indexes vector must have the same length. "
          "Angles vector has length %zu, while indexes vector has length %zu",
          cmd->positions.size(), cmd->indexes.size());
        return;
      }

      for (unsigned i = 0; i < cmd->indexes.size(); ++i) {
        int index = cmd->indexes.at(i);
        float position = cmd->positions.at(i);
        jointPositions.positions.at(index) = position;
      }

      jointStates = conversion::toJointState(jointPositions);
      jointStates.header.stamp = now();
    });

  timer = this->create_wall_timer(
    PERIOD,
    [this]() {
      jointPositionsPub->publish(jointPositions);
      jointStatesPub->publish(jointStates);
    });
}

}  // namespace nao_loopback

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_loopback::NaoLoopback)
