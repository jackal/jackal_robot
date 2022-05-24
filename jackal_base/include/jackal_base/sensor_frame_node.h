/**
 *
 *  \file       sensor_frame_node.h
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
