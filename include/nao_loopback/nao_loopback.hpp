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

#ifndef NAO_LOOPBACK__NAO_LOOPBACK_HPP_
#define NAO_LOOPBACK__NAO_LOOPBACK_HPP_

#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace nao_loopback
{

class NaoLoopback : public rclcpp::Node
{
public:
  explicit NaoLoopback(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr jointPositionsPub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jointPositionsSub;
  rclcpp::TimerBase::SharedPtr timer;

  nao_lola_sensor_msgs::msg::JointPositions jointPositions;
  sensor_msgs::msg::JointState jointStates;
};

}  // namespace nao_loopback

#endif  // NAO_LOOPBACK__NAO_LOOPBACK_HPP_
