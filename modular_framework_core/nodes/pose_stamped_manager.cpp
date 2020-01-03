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


#include <modular_framework_core/pose_stamped_manager.hpp>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
PoseStampedManager::PoseStampedManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Define the values corresponding to the number of anonymous (nameless) pose stamped stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_pose_service_ = node_handler_.advertiseService("add_pose_stamped", &PoseStampedManager::_add_pose, this);
    retrieve_pose_service_ = node_handler_.advertiseService("get_pose_stamped", &PoseStampedManager::_get_pose, this);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The pose stamped manager is ready");
}

/**
 Store a PoseStamped
 * @param  request  Object containing a field "pose_name" (string) that can be empty
                    and "pose_stamped" (geometry_msgs::PoseStamped)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool PoseStampedManager::_add_pose(modular_framework_core::AddPoseStampedRequest& request,
                                   modular_framework_core::AddPoseStampedResponse& response)
{
    // Extract the objects from the request
    std::string pose_name = request.pose_name;
    geometry_msgs::PoseStamped pose_stamped = request.pose_stamped;
    // If a pose with the same name already exists, then display an info message stating that it will be overwritten
    if (poses_map_.count(pose_name) >= 1)
    {
        ROS_INFO_STREAM("The pose stamped named " << pose_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (pose_name.empty())
    {
        anonymous_poses_.push_back(pose_stamped);
        // Increment the number of stored poses
        anonymous_stored_index_++;
    }
    // Otherwise just store it in the map
    else
    {
        poses_map_[pose_name] = pose_stamped;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored PoseStamped message
 * @param  request  Object containing a field "pose_name" (string) depicting the name of the pose to retrieve.
                    If the field is empty then returns the oldest element of the anonymous poses
 * @param  response Object containing a field "pose_stamped" (geometry_msgs::PoseStamped) containing the requested pose
                    and a field "success" (boolean) stating whether the pose can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool PoseStampedManager::_get_pose(modular_framework_core::GetPoseStampedRequest& request,
                                   modular_framework_core::GetPoseStampedResponse& response)
{
    // Extract the information from the request
    std::string pose_name = request.pose_name;
    // If the name of the pose is empty, and the anonymous pose vector still has elements then return the oldest
    // element
    if ((pose_name.empty()) and (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the pose to the oldest element and delete it in order to save memory
        response.pose_stamped = anonymous_poses_[anonymous_requested_index_];
        anonymous_poses_.erase(anonymous_poses_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous poses then display an error
    // and set the success field to false
    else if ((anonymous_requested_index_ >= anonymous_stored_index_) and (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of poses saved!");
        response.success = false;
        return true;
    }
    // If a pose message with the requested name is in the map then access it and set it to the response
    else if (poses_map_.count(pose_name) == 1)
    {
        response.pose_stamped = poses_map_[pose_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The pose named " << pose_name << " does not seem to exist!");
        response.success = false;
        return true;
    }
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_stamped_manager_server");
    ros::NodeHandle node_handle;
    PoseStampedManager pose_stamped_manager(&node_handle);
    ros::spin();
    return 0;
}
