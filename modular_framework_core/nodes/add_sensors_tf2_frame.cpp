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

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>

// Node creating the frames corresponding to sensors and add the tranform to the tf2 tree
int main(int argc, char** argv)
{
    // Initializing the node
    ros::init(argc, argv, "sensors_tf2_broadcaster");
    ros::NodeHandle node_handler;

    YAML::Node sensors = YAML::LoadFile(argv[1]);
    for (YAML::const_iterator sensor = sensors.begin(); sensor != sensors.end();
         ++sensor)
    {
        // If the parsed object is a map then it can be a sensor configuration
        if (sensor->second.Type() == YAML::NodeType::Map)
        {
            // The map must contain all these fields
            if ( !(sensor->second["origin_frame_id"] && sensor->second["frame_id"] && sensor->second["frame_x"] &&
                   sensor->second["frame_y"] && sensor->second["frame_z"] && sensor->second["frame_roll"] &&
                   sensor->second["frame_pitch"] && sensor->second["frame_yaw"]))
            {
                ROS_WARN_STREAM("Can not initialize " << sensor->first.as<std::string>() <<
                                " in the tf2 tree since an element is missing.");
            }
            else
            {
                // Define the tf message and broadcaster
                static tf2_ros::StaticTransformBroadcaster tf2_broadcaster;
                geometry_msgs::TransformStamped transform_message;

                // Fill the message
                transform_message.header.stamp = ros::Time::now();
                // Store the information provided in the YAML file
                transform_message.header.frame_id = sensor->second["origin_frame_id"].as<std::string>();
                transform_message.child_frame_id = sensor->second["frame_id"].as<std::string>();
                transform_message.transform.translation.x = sensor->second["frame_x"].as<float>();
                transform_message.transform.translation.y = sensor->second["frame_y"].as<float>();
                transform_message.transform.translation.z = sensor->second["frame_z"].as<float>();
                // Define a quaternion and set its value given RPY provided in the YAML file
                tf2::Quaternion quaternion;
                float roll_angle = sensor->second["frame_roll"].as<float>();
                float pitch_angle = sensor->second["frame_pitch"].as<float>();
                float yaw_angle = sensor->second["frame_yaw"].as<float>();
                quaternion.setRPY(roll_angle, pitch_angle, yaw_angle);
                // Fill orientation of the frame
                transform_message.transform.rotation.x = quaternion.x();
                transform_message.transform.rotation.y = quaternion.y();
                transform_message.transform.rotation.z = quaternion.z();
                transform_message.transform.rotation.w = quaternion.w();

                tf2_broadcaster.sendTransform(transform_message);
            }
        }
    }
    ros::spin();
    return 0;
}
