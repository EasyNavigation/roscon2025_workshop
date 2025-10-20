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

#ifndef EASYNAV_PLANNER__WORKSHOPPLANNER_HPP_
#define EASYNAV_PLANNER__WORKSHOPPLANNER_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

  // Define a structure to hold path information
  struct PathInfo
  {
    geometry_msgs::msg::Point origin;
    double radius;
    int num_waypoints;
  };

  class WorkshopPlanner : public PlannerMethodBase
  {
  public:
    explicit WorkshopPlanner();
    virtual std::expected<void, std::string> on_initialize() override;
    void update(NavState &nav_state) override;
    nav_msgs::msg::Path create_circular_path(
        const nav_msgs::msg::Odometry &robot_pose,
        double radius,
        int num_waypoints,
        const std::string &frame_id);

  protected:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path current_path_;
    double path_radius_;
    int path_wp_;
  };

} // namespace easynav

#endif // EASYNAV_PLANNER__WORKSHOPPLANNER_HPP_
