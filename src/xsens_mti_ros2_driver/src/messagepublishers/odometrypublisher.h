//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.

#ifndef ODOMETRYPUBLISHER_H
#define ODOMETRYPUBLISHER_H

#include "packetcallback.h"
#include "local_enu.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif
#include <stdexcept>

// A GNSS/INS measurement of the sensor, not a continuous vehicle odometry source.
struct ODOMETRYPublisher : public PacketCallback
{
    DriverPublisher<nav_msgs::msg::Odometry> pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    std::string odometry_frame_id = "local_enu";
    bool publish_tf = false;
    xsens::LocalEnu local;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

    ODOMETRYPublisher(DriverNode::SharedPtr node)
    {
        int queue_size = 5;
        node->get_parameter("publisher_queue_size", queue_size);
        node->get_parameter("frame_id", frame_id);
        node->get_parameter("odometry_frame_id", odometry_frame_id);
        node->get_parameter("pub_odometry_tf", publish_tf);
        if (frame_id.empty() || odometry_frame_id.empty() || frame_id == odometry_frame_id)
            throw std::invalid_argument("Odometry parent and sensor frame IDs must be nonempty and distinct");
        pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry", queue_size);
        if (publish_tf)
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsPositionLLA() || !packet.containsOrientation() ||
            !packet.containsCalibratedGyroscopeData() || !packet.containsVelocity())
            return;

        const auto p = packet.positionLLA();
        // Request ENU explicitly, including when the device outputs NED or NWU.
        const auto q = packet.orientationQuaternion(XDI_CoordSysEnu);
        const auto v = packet.velocity(XDI_CoordSysEnu);
        const auto g = packet.calibratedGyroscopeData();
        Eigen::Quaterniond attitude(q.w(), q.x(), q.y(), q.z());
        const Eigen::Vector3d velocity(v[0], v[1], v[2]);
        const Eigen::Vector3d gyro(g[0], g[1], g[2]);
        if (!xsens::LocalEnu::valid(p[0], p[1], p[2]) ||
            !attitude.coeffs().allFinite() || attitude.norm() < 1e-12 ||
            !velocity.allFinite() || !gyro.allFinite())
            return;
        attitude.normalize();
        if (!local.initialized())
            local.reset(p[0], p[1], p[2]);

        const auto position = local.position(p[0], p[1], p[2]);
        // Device attitude is relative to ENU at the current position. Express it
        // in the fixed ENU frame at startup, just like the position.
        const Eigen::Quaterniond orientation = local.orientation(p[0], p[1], attitude);
        // nav_msgs/Odometry requires twist in child_frame_id (sensor/body axes).
        const Eigen::Vector3d body_velocity = attitude.conjugate() * velocity;
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = odometry_frame_id;
        msg.child_frame_id = frame_id;
        msg.pose.pose.position.x = position.x();
        msg.pose.pose.position.y = position.y();
        msg.pose.pose.position.z = position.z();
        msg.pose.pose.orientation.w = orientation.w();
        msg.pose.pose.orientation.x = orientation.x();
        msg.pose.pose.orientation.y = orientation.y();
        msg.pose.pose.orientation.z = orientation.z();
        msg.twist.twist.linear.x = body_velocity.x();
        msg.twist.twist.linear.y = body_velocity.y();
        msg.twist.twist.linear.z = body_velocity.z();
        msg.twist.twist.angular.x = gyro.x();
        msg.twist.twist.angular.y = gyro.y();
        msg.twist.twist.angular.z = gyro.z();
        pub->publish(msg);

        if (broadcaster)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg.header;
            tf.child_frame_id = msg.child_frame_id;
            tf.transform.translation.x = position.x();
            tf.transform.translation.y = position.y();
            tf.transform.translation.z = position.z();
            tf.transform.rotation = msg.pose.pose.orientation;
            broadcaster->sendTransform(tf);
        }
    }
};
#endif
