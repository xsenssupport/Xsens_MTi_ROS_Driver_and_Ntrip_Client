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



#ifndef GNSSPUBLISHER_H
#define GNSSPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#define FIX_TYPE_2D_FIX (2)
#define FIX_TYPE_3D_FIX (3)
#define FIX_TYPE_GNSS_AND_DEAD_RECKONING (4)

struct GnssPublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    GnssPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gnss", pub_queue_size);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsRawGnssPvtData())
        {
            sensor_msgs::msg::NavSatFix msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsRawGnssPvtData gnss = packet.rawGnssPvtData();

            msg.latitude = (double)gnss.m_lat * 1e-7;
            msg.longitude = (double)gnss.m_lon * 1e-7;
            msg.altitude = (double)gnss.m_height * 1e-3;
            // Position covariance [m^2], ENU
            double sh = ((double)gnss.m_hAcc * 1e-3);
            double sv = ((double)gnss.m_vAcc * 1e-3);
            msg.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            switch (gnss.m_fixType)
            {
            case FIX_TYPE_2D_FIX: // fall through
            case FIX_TYPE_3D_FIX: // fall through
            case FIX_TYPE_GNSS_AND_DEAD_RECKONING:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                break;
            default:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }
            msg.status.service = 0; // unknown

            pub->publish(msg);
        }
    }
};

#endif
