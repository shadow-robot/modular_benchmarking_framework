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


#ifndef MODULAR_FRAMEWORK_CORE_POSE_STAMPED_MANAGER_H
#define MODULAR_FRAMEWORK_CORE_POSE_STAMPED_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <modular_framework_core/AddPoseStamped.h>
#include <modular_framework_core/GetPoseStamped.h>
#include <string>
#include <vector>
#include <map>

/**
 Class containing all methods and attributes required to create a pose stamped manager
 */
class PoseStampedManager
{
  public:
    // Constructor
    explicit PoseStampedManager(ros::NodeHandle* nodehandle);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named poses
    std::map<std::string, geometry_msgs::PoseStamped> poses_map_;
    // Vector gathering all nameless poses
    std::vector<geometry_msgs::PoseStamped> anonymous_poses_;
    // Service server for adding new pose
    ros::ServiceServer add_pose_service_;
    // Service server for retrieving  a pose
    ros::ServiceServer retrieve_pose_service_;
    // Indices used to keep track of the anonymous poses registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve poses, that are going to be linked to the services
    bool _add_pose(modular_framework_core::AddPoseStampedRequest& request,
                   modular_framework_core::AddPoseStampedResponse& response);
    bool _get_pose(modular_framework_core::GetPoseStampedRequest& request,
                   modular_framework_core::GetPoseStampedResponse& response);
};

#endif  // MODULAR_FRAMEWORK_CORE_POSE_STAMPED_MANAGER_H
