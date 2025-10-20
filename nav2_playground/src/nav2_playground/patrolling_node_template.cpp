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

#include "nav2_playground/patrolling_node_template.hpp"
#include <chrono>

namespace nav2_playground
{

using namespace std::chrono_literals;

PatrollingNode::PatrollingNode(const rclcpp::NodeOptions & options)
: Node("patrolling_node", options)
{
  timer_ = create_wall_timer(
    100ms,
    std::bind(&PatrollingNode::cycle, this));
}

void PatrollingNode::initialize()
{
  // Load waypoints parameter
  std::vector<std::string> waypoint_names;
  declare_parameter("waypoints", waypoint_names);
  get_parameter("waypoints", waypoint_names);

  declare_parameter<std::string>("frame_id", "map");
  get_parameter("frame_id", frame_id_);

  if (waypoint_names.empty()) {
    RCLCPP_ERROR(get_logger(), "No waypoints provided in 'waypoints' parameter");
    return;
  }

  RCLCPP_INFO(get_logger(), "Loading %zu waypoints", waypoint_names.size());

  // Load each waypoint's coordinates
  for (const auto & wp_name : waypoint_names) {
    std::vector<double> wp_coord;
    declare_parameter(wp_name, wp_coord);
    get_parameter(wp_name, wp_coord);

    if (wp_coord.size() != 3) {
      RCLCPP_ERROR(
        get_logger(),
        "Waypoint '%s' has wrong size %zu (expected 3: x, y, yaw)",
        wp_name.c_str(), wp_coord.size());
      continue;
    }

    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = frame_id_;
    waypoint.header.stamp = now();
    waypoint.pose.position.x = wp_coord[0];
    waypoint.pose.position.y = wp_coord[1];
    waypoint.pose.position.z = 0.0;
    waypoint.pose.orientation = orientationAroundZAxis(wp_coord[2]);

    waypoints_.push_back(waypoint);
    RCLCPP_INFO(
      get_logger(), "Loaded waypoint '%s': [%.2f, %.2f, %.2f rad]",
      wp_name.c_str(), wp_coord[0], wp_coord[1], wp_coord[2]);
  }

  if (waypoints_.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid waypoints loaded. Cannot patrol.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Successfully loaded %zu valid waypoints", waypoints_.size());
}

void PatrollingNode::cycle()
{
  switch (state_) {
    case PatrolState::IDLE:
      if (!initialized_) {
        RCLCPP_INFO(get_logger(), "Initializing patrolling node");

        // Create Nav2 action client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
          this,
          "navigate_to_pose");

        initialize();
        initialized_ = true;

        if (waypoints_.empty()) {
          RCLCPP_ERROR(get_logger(), "Cannot start patrol with no waypoints");
          state_ = PatrolState::ERROR;
          break;
        }
      }

      // Wait for action server
      if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for navigate_to_pose action server...");
        break;
      }

      RCLCPP_INFO(get_logger(), "Starting patrol with %zu waypoints", waypoints_.size());
      current_waypoint_index_ = 0;
      state_ = PatrolState::SENDING_GOAL;
      break;

    case PatrolState::SENDING_GOAL:
      {
        if (current_waypoint_index_ >= waypoints_.size()) {
          RCLCPP_INFO(get_logger(), "All waypoints visited");
          state_ = PatrolState::FINISHED;
          return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_waypoint_index_];
        goal_msg.pose.header.stamp = now();

        RCLCPP_INFO(
        get_logger(), "Sending goal %zu/%zu: [%.2f, %.2f]",
        current_waypoint_index_ + 1, waypoints_.size(),
        goal_msg.pose.pose.position.x,
        goal_msg.pose.pose.position.y);

        current_future_goal_handle_ = nav_client_->async_send_goal(goal_msg);
        state_ = PatrolState::NAVIGATING;
        break;
      }

    case PatrolState::NAVIGATING:
      {
        if (!current_future_goal_handle_.valid()) {
          RCLCPP_ERROR(get_logger(), "No goal handle available");
          state_ = PatrolState::ERROR;
          break;
        }

        current_goal_handle_ = current_future_goal_handle_.get();
        if (!current_goal_handle_) {
          RCLCPP_ERROR(get_logger(), "Goal was not accepted by the action server");
          state_ = PatrolState::ERROR;
          break;
        }

        auto status = current_goal_handle_->get_status();

        switch (status) {
          case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
            RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Goal accepted, waiting to start execution...");
            break;

          case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Navigating to waypoint %zu/%zu...",
            current_waypoint_index_ + 1, waypoints_.size());
            break;

          case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(
            get_logger(), "Successfully reached waypoint %zu/%zu",
            current_waypoint_index_ + 1, waypoints_.size());
            current_goal_handle_.reset();
            state_ = PatrolState::DO_SOMETHING_AT_WAYPOINT;
            break;

          case action_msgs::msg::GoalStatus::STATUS_ABORTED:
            RCLCPP_ERROR(
            get_logger(), "Navigation was aborted by the action server");
            state_ = PatrolState::ERROR;
            break;

          case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_ERROR(
            get_logger(), "Navigation was canceled");
            state_ = PatrolState::ERROR;
            break;

          default:
            RCLCPP_ERROR(
            get_logger(), "Unexpected goal status: %d", status);
            state_ = PatrolState::ERROR;
            break;
        }
        break;
      }

    case PatrolState::DO_SOMETHING_AT_WAYPOINT:
      {
      // Implement what the robot should do when it reaches a waypoint.
      // Ideas:
      // After completing the task, you should:
      //   1. Increment current_waypoint_index_
      //   2. Transition to PatrolState::SENDING_GOAL
      //
      // YOUR CODE HERE

        break;
      }

    case PatrolState::FINISHED:
      RCLCPP_INFO(get_logger(), "Patrol cycle completed. Restarting from first waypoint.");
      current_waypoint_index_ = 0;
      state_ = PatrolState::SENDING_GOAL;
      break;

    case PatrolState::ERROR:
      // Stay in error state (could add recovery logic here)
      break;
  }
}

}  // namespace nav2_playground
