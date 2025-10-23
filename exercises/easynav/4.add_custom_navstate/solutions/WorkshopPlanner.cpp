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
          oss << "Generated circular path:\n"
              << "  Origin: (" << std::fixed << std::setprecision(2)
              << value.origin.x << ", " << value.origin.y << ", " << value.origin.z << ")\n"
              << "  Radius: " << std::fixed << std::setprecision(2) << value.radius << " m\n"
              << "  Waypoints: " << value.num_waypoints;
          return oss.str();
        });
  }

  std::expected<void, std::string>
  WorkshopPlanner::on_initialize()
  {
    auto node = get_node();
    const auto &plugin_name = get_plugin_name();

    node->declare_parameter<double>(plugin_name + ".path_radius", 0.5);
    node->declare_parameter<int>(plugin_name + ".path_wp", 16);

    node->get_parameter(plugin_name + ".path_radius", path_radius_);
    node->get_parameter(plugin_name + ".path_wp", path_wp_);

    path_pub_ = get_node()->create_publisher<nav_msgs::msg::Path>(
        node->get_fully_qualified_name() + std::string("/") + "costmap_planner/path", 10);

    return {};
  }

  nav_msgs::msg::Path
  WorkshopPlanner::create_circular_path(
      const nav_msgs::msg::Odometry &robot_pose,
      double radius,
      int num_waypoints,
      const std::string &frame_id)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = get_node()->now();

    // Extract current yaw from robot's quaternion
    double robot_yaw = atan2(
        2.0 * (robot_pose.pose.pose.orientation.w * robot_pose.pose.pose.orientation.z +
               robot_pose.pose.pose.orientation.x * robot_pose.pose.pose.orientation.y),
        1.0 - 2.0 * (robot_pose.pose.pose.orientation.y * robot_pose.pose.pose.orientation.y +
                     robot_pose.pose.pose.orientation.z * robot_pose.pose.pose.orientation.z));

    // Calculate circle center perpendicular to robot's heading (to the left)
    // This places the robot ON the circle, not at the center
    double center_x = robot_pose.pose.pose.position.x - radius * sin(robot_yaw);
    double center_y = robot_pose.pose.pose.position.y + radius * cos(robot_yaw);

    // Calculate the starting angle (where the robot currently is on the circle)
    double start_angle = atan2(
        robot_pose.pose.pose.position.y - center_y,
        robot_pose.pose.pose.position.x - center_x);

    // Generate waypoints along the circle
    for (int i = 0; i < num_waypoints; ++i)
    {
      double angle = start_angle + (2.0 * M_PI * i / num_waypoints);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = path.header.stamp;

      // Calculate position on circle (robot is ON the circle perimeter)
      pose.pose.position.x = center_x + radius * cos(angle);
      pose.pose.position.y = center_y + radius * sin(angle);
      pose.pose.position.z = robot_pose.pose.pose.position.z;

      // Calculate orientation tangent to circle (perpendicular to radius)
      double tangent_angle = angle + M_PI / 2.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sin(tangent_angle / 2.0);
      pose.pose.orientation.w = cos(tangent_angle / 2.0);

      path.poses.push_back(pose);
    }

    return path;
  }

  void
  WorkshopPlanner::update(NavState &nav_state)
  {
    // DONE: Check  robot_pose is available in nav_state
    if (!nav_state.has("robot_pose"))
    {
      return;
    }
    // DONE: get robot_pose from nav_state (type nav_msgs::msg::Odometry)
    const auto robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
    ///////////////////////////

    current_path_ = create_circular_path(robot_pose, path_radius_, path_wp_, "map");

    // Publish the path for visualization
    if (path_pub_)
    {
      path_pub_->publish(current_path_);
    }

    // DONE: Add the current path to the NavState
    nav_state.set("path", current_path_);

    // DONE: Create and store PathInfo
    PathInfo path_info;
    path_info.origin = robot_pose.pose.pose.position;
    path_info.radius = path_radius_;
    path_info.num_waypoints = path_wp_;
    nav_state.set("path_info", path_info);
    //////////////////////////

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
