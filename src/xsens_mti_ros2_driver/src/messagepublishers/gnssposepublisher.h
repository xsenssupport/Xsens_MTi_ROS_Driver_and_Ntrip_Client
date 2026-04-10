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


#ifndef GNSSPOSEPUBLISHER_H
#define GNSSPOSEPUBLISHER_H

#include "packetcallback.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

struct GNSSPOSEPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    GNSSPOSEPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;

        node->get_parameter("publisher_queue_size", pub_queue_size);
        node->get_parameter("frame_id", frame_id);

        pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_pose", pub_queue_size);

        
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsPositionLLA() && packet.containsOrientation())
        {
            geometry_msgs::msg::PoseStamped msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsVector p = packet.positionLLA();
            // publishing Lat/Long/Altitude as x,y z
            msg.pose.position.x = p[0];
            msg.pose.position.y = p[1];
            msg.pose.position.z = p[2];

            XsQuaternion q = packet.orientationQuaternion();
            // publishing Orientation as w,x,y,z
            msg.pose.orientation.w = q.w();
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();

            pub->publish(msg);
        }
    }
};

#endif
