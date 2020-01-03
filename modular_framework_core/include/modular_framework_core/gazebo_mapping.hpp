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

#ifndef MODULAR_FRAMEWORK_CORE_GAZEBO_MAPPING_H
#define MODULAR_FRAMEWORK_CORE_GAZEBO_MAPPING_H

#include <ros/ros.h>
#include <modular_framework_core/AddGazeboMapping.h>
#include <modular_framework_core/GetGazeboType.h>
#include <modular_framework_core/DeleteGazeboMapping.h>
#include <map>
#include <string>

/**
 Class containing all methods and attributes required to store and access mappings between name given to gazebo objects
 */
class GazeboMappingManager
{
  public:
    // Constructor
    explicit GazeboMappingManager(ros::NodeHandle* nodehandle);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all mapping between named objects and their type
    std::map<std::string, std::string> gazebo_name_map_;
    // Service server for adding a new mapping
    ros::ServiceServer add_gazebo_service_;
    // Service server for retrieving an object type provided its name
    ros::ServiceServer get_gazebo_object_type_service_;
    // Service server for deleting a mapping
    ros::ServiceServer delete_gazebo_mapping_service_;

    // Internal functions allowing to add, retrieve ore delete mappings or object types, that are going to be linked to
    // the services
    bool _add_gazebo_mapping(modular_framework_core::AddGazeboMappingRequest& request,
                             modular_framework_core::AddGazeboMappingResponse& response);
    bool _get_object_type(modular_framework_core::GetGazeboTypeRequest& request,
                          modular_framework_core::GetGazeboTypeResponse& response);
    bool _delete_gazebo_mapping(modular_framework_core::DeleteGazeboMappingRequest& request,
                                modular_framework_core::DeleteGazeboMappingResponse& response);
};

#endif  // MODULAR_FRAMEWORK_CORE_GAZEBO_MAPPING_H
