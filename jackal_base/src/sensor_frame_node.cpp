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

#include <jackal_base/sensor_frame_node.h>

SensorFrameNode::SensorFrameNode() :
      nh_private_("~")
{
    imu_sub_ = nh_private_.subscribe("imu/data_raw", 10,
                                      &SensorFrameNode::imuCallback, this);
    navsat_fix_sub_ = nh_private_.subscribe("navsat/fix", 10,
                                         &SensorFrameNode::navSatFixCallback, this);
    navsat_vel_sub_ = nh_private_.subscribe("navsat/vel", 10,
                                         &SensorFrameNode::navSatVelCallback, this);

    imu_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu/data", 10);
    navsat_fix_pub_ = nh_private_.advertise<sensor_msgs::NavSatFix>("navsat/fix", 10);
    navsat_vel_pub_ = nh_private_.advertise<geometry_msgs::TwistStamped>("navsat/vel", 10);
}


void SensorFrameNode::imuCallback(sensor_msgs::Imu imu_raw)
{
    addNamespaceToFrameID(&imu_raw.header.frame_id);

    imu_pub_.publish(imu_raw);
}


void SensorFrameNode::navSatFixCallback(sensor_msgs::NavSatFix navsat_fix_raw)
{
    addNamespaceToFrameID(&navsat_fix_raw.header.frame_id);

    navsat_fix_pub_.publish(navsat_fix_raw);
}


void SensorFrameNode::navSatVelCallback(geometry_msgs::TwistStamped navsat_vel_raw)
{
    addNamespaceToFrameID(&navsat_vel_raw.header.frame_id);

    navsat_vel_pub_.publish(navsat_vel_raw);
}


void SensorFrameNode::addNamespaceToFrameID(std::string *frame_id)
{
    assert(frame_id);

    const std::string ns = nh_private_.getNamespace();

    if (!ns.empty()) {
        *frame_id = ns + std::string("/") + *frame_id;
    }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "sensor_frame_node");

  SensorFrameNode node();
  ros::spin();

  return 0;
}