/*
 * Copyright 2020 Shadow Robot Company Ltd.
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

#ifndef MODULAR_FRAMEWORK_CORE_ROBOT_POSE_MANAGER_H
#define MODULAR_FRAMEWORK_CORE_ROBOT_POSE_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <modular_framework_core/AddRobotPose.h>
#include <modular_framework_core/GetRobotPose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>

/**
 Class containing all methods and attributes required to create a robot_pose manager
 */
class RobotPoseManager
{
  public:
    // Constructor
    RobotPoseManager(ros::NodeHandle* nodehandle, std::string file_path);

    // Method loading the robot poses already defined in the corresponding YAML file
    void load_robot_poses_from_file(std::string filename);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named robot poses
    std::map<std::string, modular_framework_core::RobotPose> robot_poses_map_;
    // Vector gathering all nameless robot poses
    std::vector<modular_framework_core::RobotPose> anonymous_robot_poses_;
    // Service server for adding new robot pose
    ros::ServiceServer add_robot_pose_service_;
    // Service server for retrieving a  robot pose
    ros::ServiceServer retrieve_robot_pose_service_;
    // Indices used to keep track of the anonymous robot poses registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve robot poses, that are going to be linked to the services
    bool _add_robot_pose(modular_framework_core::AddRobotPoseRequest& request,
                         modular_framework_core::AddRobotPoseResponse& response);
    bool _get_robot_pose(modular_framework_core::GetRobotPoseRequest& request,
                         modular_framework_core::GetRobotPoseResponse& response);
};

#endif  // MODULAR_FRAMEWORK_CORE_ROBOT_POSE_MANAGER_H
