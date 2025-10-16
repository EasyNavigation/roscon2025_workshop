// Copyright 2025 Intelligent Robotics Lab
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef NAV2_PLAYGROUND__PATROLLING_NODE_TEMPLATE_HPP_
#define NAV2_PLAYGROUND__PATROLLING_NODE_TEMPLATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_playground
{

inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  return tf2::toMsg(q);
}

class PatrollingNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RCLCPP_SMART_PTR_DEFINITIONS(PatrollingNode)

  explicit PatrollingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~PatrollingNode() = default;

private:
  void initialize();
  void cycle();

  enum class PatrolState
  {
    IDLE, SENDING_GOAL, NAVIGATING, DO_SOMETHING_AT_WAYPOINT, FINISHED, ERROR
  };
  PatrolState state_ {PatrolState::IDLE};

  bool initialized_ {false};
  size_t current_waypoint_index_ {0};

  std::string frame_id_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
  std::shared_future<GoalHandleNavigateToPose::SharedPtr> current_future_goal_handle_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_playground

#endif  // NAV2_PLAYGROUND__PATROLLING_NODE_TEMPLATE_HPP_
