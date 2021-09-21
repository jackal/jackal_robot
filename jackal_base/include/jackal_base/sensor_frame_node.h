/*
 *  Copyright (C) 2021, The University of Texas at Austin
 *  Robert Blake Anderson <blakeanderson@utexas.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JACKAL_BASE_SENSOR_FRAME_NODE_H
#define JACKAL_BASE_SENSOR_FRAME_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

/**
 * SensorFrameNode adds namespacing for the Frame IDs
 * of sensor messages emitted by the Jackal MCU. This is
 * necessary to distinguish between identically-named
 * frames from different robots.
 *
 * Operation:
 *  1) Usage: rosrun jackal_base sensor_frame_node
 *  2) Subscribes to the imu/data_raw, navsat/fix, and navsat/vel
 *  3) Upon receiving a message on those topics, prepends the
 *     namespace of this node to the header/frame_id field.
 *     (ex. imu_link becomes jackal_two/imu_link)
 */
class SensorFrameNode
{
  public:

    SensorFrameNode();

  private:

    void imuCallback(sensor_msgs::Imu imu_raw);

    void navSatFixCallback(sensor_msgs::NavSatFix navsat_fix_raw);

    void navSatVelCallback(geometry_msgs::TwistStamped navsat_vel_raw);

    void addNamespaceToFrameID(std::string *frame_id);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber imu_sub_;
    ros::Subscriber navsat_fix_sub_;
    ros::Subscriber navsat_vel_sub_;

    ros::Publisher imu_pub_;
    ros::Publisher navsat_fix_pub_;
    ros::Publisher navsat_vel_pub_;
};

#endif // JACKAL_BASE_SENSOR_FRAME_NODE_H
