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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "easynav_system/GoalManagerClient.hpp"
#include "easynav_patrolling_behavior/PatrollingNode.hpp"

namespace easynav
{

  using namespace std::chrono_literals;

  PatrollingNode::PatrollingNode(const rclcpp::NodeOptions &options)
      : Node("patrolling_node", options)
  {
    timer_ = create_timer(
        100ms,
        std::bind(&PatrollingNode::cycle, this));
  }

  void
  PatrollingNode::initialize()
  {
    std::vector<std::string> waypoints;
    declare_parameter("waypoints", waypoints);
    get_parameter("waypoints", waypoints);

    declare_parameter<std::string>("frame_id", "map");
    get_parameter("frame_id", frame_id_);

    goals_.header.frame_id = frame_id_;

    for (const auto &wp : waypoints)
    {
      std::vector<double> wp_coord;
      declare_parameter(wp, wp_coord);
      get_parameter(wp, wp_coord);

      if (wp_coord.size() != 3)
      {
        RCLCPP_ERROR(get_logger(), "Coordinates for wp [%s] have wrong size %zu",
                     wp.c_str(), wp_coord.size());
        continue;
      }

      geometry_msgs::msg::PoseStamped wp_pose;
      wp_pose.header.frame_id = frame_id_;
      wp_pose.pose.position.x = wp_coord[0];
      wp_pose.pose.position.y = wp_coord[1];
      wp_pose.pose.orientation = orientationAroundZAxis(wp_coord[2]);

      goals_.goals.push_back(wp_pose);
    }
  }

  void
  PatrollingNode::cycle()
  {
    switch (state_)
    {
    case PatrolState::IDLE:
    {
      if (!initialized_)
      {
        RCLCPP_INFO(get_logger(), "Initializing patrolling");

        gm_client_ = GoalManagerClient::make_shared(shared_from_this());
        initialize();
        initialized_ = true;
      }

      RCLCPP_INFO(get_logger(), "Goals sent");

      nav_msgs::msg::Goals single_goal;
      single_goal.header = goals_.header;
      single_goal.goals.push_back(goals_.goals[current_goal_index_]);
      gm_client_->send_goals(single_goal);
      state_ = PatrolState::PATROLLING;
    }
    break;
    case PatrolState::PATROLLING:
    {
      auto nav_state = gm_client_->get_state();
      switch (nav_state)
      {
      case GoalManagerClient::State::NAVIGATION_REJECTED:
      case GoalManagerClient::State::NAVIGATION_FAILED:
      case GoalManagerClient::State::NAVIGATION_CANCELLED:
      case GoalManagerClient::State::ERROR:
        RCLCPP_ERROR(get_logger(), "Navigation finished with error %s",
                     gm_client_->get_result().status_message.c_str());
        state_ = PatrolState::ERROR;
        break;
      case GoalManagerClient::State::NAVIGATION_FINISHED:
        RCLCPP_INFO(get_logger(), "Navigation succesfully finished with message %s",
                    gm_client_->get_result().status_message.c_str());
        pause_start_time_ = now();
        RCLCPP_INFO(get_logger(), "Waiting time started at waypoint %zu", current_goal_index_ + 1);
        state_ = PatrolState::DO_AT_WAYPOINT;
        break;
      case GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
        // RCLCPP_INFO(get_logger(), "Goals accepted and navigating");
        break;
      default:
        break;
      }
    }
    break;

    case PatrolState::DO_AT_WAYPOINT:
      if (now() - pause_start_time_ >= pause_duration_)
      {
        RCLCPP_INFO(get_logger(), "Waiting time ended at waypoint %zu", current_goal_index_ + 1);

        // advance to next waypoint
        ++current_goal_index_;
        if (current_goal_index_ < goals_.goals.size())
        {
          RCLCPP_INFO(get_logger(), "Navigating to waypoint %zu", current_goal_index_ + 1);
          // gm_client_->send_goal(goals_.goals[current_goal_index_]);
          gm_client_->reset();
          state_ = PatrolState::IDLE;
        }
        else
        {
          RCLCPP_INFO(get_logger(), "All waypoints completed");
          state_ = PatrolState::FINISHED;
        }
      }
      break;

    case PatrolState::FINISHED:
      RCLCPP_INFO(get_logger(), "Reset navigation");
      current_goal_index_ = 0;
      state_ = PatrolState::IDLE;
      if (gm_client_->get_state() != GoalManagerClient::State::IDLE)
      {
        gm_client_->reset();
      }
      break;
    case PatrolState::ERROR:
      break;
    }
  }

} // namespace easynav
