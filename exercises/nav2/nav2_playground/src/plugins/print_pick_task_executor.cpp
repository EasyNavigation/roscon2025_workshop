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

#include "nav2_playground/plugins/print_pick_task_executor.hpp"
#include <thread>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>

namespace nav2_playground
{

void PrintPickTaskExecutor::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  parent_weak_ = parent;
  plugin_name_ = plugin_name;
  auto parent_locked = parent_weak_.lock();
  if (!parent_locked) {
    RCLCPP_ERROR(logger_, "[%s] Failed to lock parent lifecycle node", plugin_name_.c_str());
    return;
  }
  logger_ = parent_locked->get_logger();
  parent_locked->declare_parameter(plugin_name_ + ".simulated_delay_ms", simulated_delay_ms_);
  parent_locked->get_parameter(plugin_name_ + ".simulated_delay_ms", simulated_delay_ms_);
  RCLCPP_INFO(logger_, "[%s] Initialized with simulated delay %d ms", plugin_name_.c_str(),
      simulated_delay_ms_);
}

bool PrintPickTaskExecutor::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose,
  const int & curr_waypoint_index)
{
  (void)curr_pose;
  RCLCPP_INFO(logger_, "[%s] Performing PICK operation at waypoint %d", plugin_name_.c_str(),
      curr_waypoint_index);
  std::this_thread::sleep_for(std::chrono::milliseconds(simulated_delay_ms_));
  RCLCPP_INFO(logger_, "[%s] PICK complete at waypoint %d", plugin_name_.c_str(),
      curr_waypoint_index);
  return true;  //
}

}  // namespace nav2_playground

PLUGINLIB_EXPORT_CLASS(nav2_playground::PrintPickTaskExecutor, nav2_core::WaypointTaskExecutor)
