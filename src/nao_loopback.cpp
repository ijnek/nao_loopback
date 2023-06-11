#include "nao_loopback/nao_loopback.hpp"

namespace nao_loopback
{

NaoLoopback::NaoLoopback(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_loopback", options}
{
  jointPositionsPub =
    create_publisher<nao_sensor_msgs::msg::JointPositions>("sensors/joint_positions", 10);

  jointPositionsSub =
    create_subscription<nao_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_command_msgs::msg::JointPositions::SharedPtr cmd) {

      if (cmd->indexes.size() != cmd->positions.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "Incorrect message received for nao_command_msgs::msg::JointPositions. "
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

      jointPositionsPub->publish(jointPositions);
    });
}

}  // namespace nao_loopback

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_loopback::NaoLoopback)
