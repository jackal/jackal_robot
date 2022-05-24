/**
 *
 *  \file       sensor_frame_node.cpp
 *  \brief      Applies namespacing to frame_ids in messages produced by the
 *              platform's MCU.
 *  \author     Blake Anderson <blakeanderson@utexas.edu>
 *  \copyright  Copyright (c) 2021, The University of Texas at Austin.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The University of Texas nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL The University of Texas BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
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