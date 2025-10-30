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

#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

#include "easynav_workshop_planner/WorkshopPlanner.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

  WorkshopPlanner::WorkshopPlanner()
  {
    NavState::register_printer<PathInfo>(
        [](const PathInfo &value)
        {
          std::ostringstream oss;
          oss << "Generated linear path:\n"
              << "  Origin: (" << std::fixed << std::setprecision(2)
              << value.origin.x << ", " << value.origin.y << ", " << value.origin.z << ")\n"
              << "  Goal: (" << std::fixed << std::setprecision(2)
              << value.goal.x << ", " << value.goal.y << ", " << value.goal.z << ")\n"
              << "  Waypoints: " << value.num_waypoints;
          return oss.str();
        });
  }

  std::expected<void, std::string>
  WorkshopPlanner::on_initialize()
  {
    auto node = get_node();
    const auto &plugin_name = get_plugin_name();

    node->declare_parameter<int>(plugin_name + ".path_wp", 16);

    node->get_parameter(plugin_name + ".path_wp", path_wp_);

    path_pub_ = get_node()->create_publisher<nav_msgs::msg::Path>(
        node->get_fully_qualified_name() + std::string("/") + "costmap_planner/path", 10);

    return {};
  }

  nav_msgs::msg::Path
  WorkshopPlanner::create_linear_path(
      const geometry_msgs::msg::Pose &robot_pose,
      const geometry_msgs::msg::Pose &goal_pose,
      const std::string &frame_id)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = get_node()->now();

    // Calculate the direction vector from robot to goal
    double dx = goal_pose.position.x - robot_pose.position.x;
    double dy = goal_pose.position.y - robot_pose.position.y;
    double dz = goal_pose.position.z - robot_pose.position.z;

    // Generate waypoints along the line
    for (int i = 0; i < path_wp_; ++i)
    {
      double t = static_cast<double>(i) / (path_wp_ - 1); // Interpolation factor [0, 1]

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = path.header.stamp;

      // Linear interpolation of position
      pose.pose.position.x = robot_pose.position.x + t * dx;
      pose.pose.position.y = robot_pose.position.y + t * dy;
      pose.pose.position.z = robot_pose.position.z + t * dz;

      // Calculate orientation pointing towards the goal
      double yaw = std::atan2(dy, dx);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);

      path.poses.push_back(pose);
    }

    return path;
  }
  void
  WorkshopPlanner::update(NavState &nav_state)
  {

    if (!nav_state.has("goals") || !nav_state.has("robot_pose"))
    {
      RCLCPP_DEBUG(get_node()->get_logger(), "goals, robot_pose or map missing. Returning");
      return;
    }
    const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
    if (goals.goals.empty())
    {
      RCLCPP_DEBUG(get_node()->get_logger(), "goals empty. Returning empty path");
      current_path_.poses.clear();
      nav_state.set("path", current_path_);
      return;
    }
    const auto &robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    const auto &goal_pose = goals.goals.front();

    // Create linear path from robot to goal
    current_path_ = create_linear_path(
        robot_pose.pose.pose,
        goal_pose.pose,
        "map");

    // Publish the path for visualization
    if (path_pub_)
    {
      path_pub_->publish(current_path_);
    }

    // Store path in nav_state
    nav_state.set("path", current_path_);

    // Create and store PathInfo
    PathInfo path_info;
    path_info.origin = robot_pose.pose.pose.position;
    path_info.goal = goal_pose.pose.position;
    path_info.num_waypoints = path_wp_;
    nav_state.set("path_info", path_info);

    // Create a temporary NavState to use the debug printer
    NavState temp_state;
    if (nav_state.has("path_info"))
    {
      temp_state.set("path_info", nav_state.get<PathInfo>("path_info"));
      RCLCPP_INFO(
          get_node()->get_logger(),
          "WorkshopPlanner:\n%s",
          temp_state.debug_string().c_str());
    }
  }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::WorkshopPlanner, easynav::PlannerMethodBase)
