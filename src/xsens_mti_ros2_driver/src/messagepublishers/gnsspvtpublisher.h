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
//


#ifndef GNSSPVTPUBLISHER_H
#define GNSSPVTPUBLISHER_H

#include "packetcallback.h"
#include <xsens_mti_ros2_driver/msg/gnss_pvt.hpp>

struct GnssPvtPublisher : public PacketCallback
{
    rclcpp::Publisher<xsens_mti_ros2_driver::msg::GnssPvt>::SharedPtr pub;
    std::string frame_id = "world";

    GnssPvtPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<xsens_mti_ros2_driver::msg::GnssPvt>("/gnss/pvt", pub_queue_size);
        node->get_parameter("fixed_frame_id", frame_id);
    }

    static xsens_mti_ros2_driver::msg::GnssPvt buildMessage(
        const XsRawGnssPvtData &pvt,
        rclcpp::Time timestamp,
        const std::string &frame_id)
    {
        xsens_mti_ros2_driver::msg::GnssPvt msg;

        msg.header.stamp    = timestamp;
        msg.header.frame_id = frame_id;

        msg.i_tow    = pvt.m_itow;
        msg.year     = pvt.m_year;
        msg.month    = pvt.m_month;
        msg.day      = pvt.m_day;
        msg.hour     = pvt.m_hour;
        msg.min      = pvt.m_min;
        msg.sec      = pvt.m_sec;
        msg.valid    = pvt.m_valid;
        msg.t_acc    = pvt.m_tAcc;
        msg.nano     = pvt.m_nano;
        msg.fix_type = pvt.m_fixType;
        msg.flags    = pvt.m_flags;
        msg.num_sv   = pvt.m_numSv;
        msg.lon      = pvt.m_lon;
        msg.lat      = pvt.m_lat;
        msg.height   = pvt.m_height;
        msg.h_msl    = pvt.m_hMsl;
        msg.h_acc    = pvt.m_hAcc;
        msg.v_acc    = pvt.m_vAcc;
        msg.vel_n    = pvt.m_velN;
        msg.vel_e    = pvt.m_velE;
        msg.vel_d    = pvt.m_velD;
        msg.g_speed  = pvt.m_gSpeed;
        msg.heading  = pvt.m_headMot;
        msg.s_acc    = pvt.m_sAcc;
        msg.head_acc = pvt.m_headAcc;
        msg.head_veh = pvt.m_headVeh;
        msg.p_dop    = pvt.m_pdop;

        return msg;
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsRawGnssPvtData())
        {
            XsRawGnssPvtData pvt = packet.rawGnssPvtData();
            pub->publish(buildMessage(pvt, timestamp, frame_id));
        }
    }
};

#endif
