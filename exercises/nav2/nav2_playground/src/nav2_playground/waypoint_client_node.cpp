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

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  return tf2::toMsg(q);
}


class WaypointClientNode : public rclcpp::Node {
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  WaypointClientNode()
  : Node("waypoint_client_node")
  {
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});
    declare_parameter<int>("loops", 1);

    loops_remaining_ = get_parameter("loops").as_int();

    load_waypoints_from_params();
    action_client_ = rclcpp_action::create_client<FollowWaypoints>(
      this,
      "follow_waypoints");

    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints loaded from parameters.");
      return;
    }

    send_goal();
  }

private:
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  int loops_remaining_ {1};
  rclcpp::TimerBase::SharedPtr restart_timer_;

  void load_waypoints_from_params()
  {
    waypoints_.clear();

    // Load waypoints parameter
    std::vector<std::string> waypoint_names;
    get_parameter("waypoints", waypoint_names);

    std::string frame_id;
    get_parameter("frame_id", frame_id);

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
      waypoint.header.frame_id = frame_id;
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
      RCLCPP_ERROR(get_logger(), "No valid waypoints loaded.");
    } else {
      RCLCPP_INFO(get_logger(), "Successfully loaded %zu valid waypoints", waypoints_.size());
    }
  }

  void send_goal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "FollowWaypoints action server not available");
      return;
    }

    FollowWaypoints::Goal goal_msg;
    goal_msg.poses = waypoints_;

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleFollowWaypoints> gh) {
        if (!gh) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Current waypoint index: %u", feedback->current_waypoint);
      };
    send_goal_options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result->missed_waypoints.empty()) {
              RCLCPP_INFO(this->get_logger(), "Successfully visited all waypoints");
            } else {
              RCLCPP_WARN(this->get_logger(), "Missed %zu waypoints",
            result.result->missed_waypoints.size());
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointClientNode>();
  rclcpp::spin(node);
  return 0;
}
