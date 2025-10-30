// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#ifndef EASYNAV_PATROLLING__BEHAVIOR_PATROLLING_NODE__HPP_
#define EASYNAV_PATROLLING__BEHAVIOR_PATROLLING_NODE__HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_system/GoalManagerClient.hpp"

namespace easynav
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
  RCLCPP_SMART_PTR_DEFINITIONS(PatrollingNode)

  explicit PatrollingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PatrollingNode() = default;

private:
  void initialize();
  void cycle();
  nav_msgs::msg::Goals build_current_goal();
  enum class PatrolState {IDLE, PATROLLING, FINISHED, ERROR, DO_AT_WAYPOINT};
  PatrolState state_ {PatrolState::IDLE};

  bool initialized_ {false};
  size_t send_retries_ {0};
  const size_t max_retries_ {3};
  uint last_control_type_ {0};

  std::string frame_id_;
  nav_msgs::msg::Goals goals_;
  GoalManagerClient::SharedPtr gm_client_;
  rclcpp::Time pause_start_time_;
  rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(2.0);
  std::size_t current_goal_index_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace easynav

#endif  // EASYNAV_PATROLLING__BEHAVIOR_PATROLLING_NODE__HPP_
