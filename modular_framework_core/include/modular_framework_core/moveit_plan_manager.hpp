/*
* Copyright 2019 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODULAR_FRAMEWORK_CORE_MOVEIt_PLAN_MANAGER_H
#define MODULAR_FRAMEWORK_CORE_MOVEIt_PLAN_MANAGER_H

#include <ros/ros.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <modular_framework_core/AddMoveitPlan.h>
#include <modular_framework_core/GetMoveitPlan.h>
#include <string>
#include <vector>
#include <map>

/**
 Class containing all methods and attributes required to create a pose stamped manager
 */
class MoveitPlanManager
{
  public:
    // Constructor
    MoveitPlanManager(ros::NodeHandle* nodehandle);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named moveit plans
    std::map<std::string, moveit_msgs::RobotTrajectory> plans_map_;
    // Vector gathering all nameless moveit plans
    std::vector<moveit_msgs::RobotTrajectory> anonymous_plans_;
    // Service server for adding new moveit plan
    ros::ServiceServer add_plan_service_;
    // Service server for retrieving a moveit plan
    ros::ServiceServer retrieve_plan_service_;
    // Indices used to keep track of the anonymous plans registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve poses, that are going to be linked to the services
    bool _add_plan(modular_framework_core::AddMoveitPlanRequest& request,
                   modular_framework_core::AddMoveitPlanResponse& response);
    bool _get_plan(modular_framework_core::GetMoveitPlanRequest& request,
                   modular_framework_core::GetMoveitPlanResponse& response);
};

#endif  // MODULAR_FRAMEWORK_CORE_MOVEIt_PLAN_MANAGER_H
