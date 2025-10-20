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

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_playground
{

// Simple task executor that mimics a 'pick' operation at a waypoint.
// For demonstration, it just logs messages and simulates some work delay.
class PrintPickTaskExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  PrintPickTaskExecutor() = default;
  ~PrintPickTaskExecutor() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_weak_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PrintPickTaskExecutor")};
  int simulated_delay_ms_ {1000};
};

}  // namespace nav2_playground
