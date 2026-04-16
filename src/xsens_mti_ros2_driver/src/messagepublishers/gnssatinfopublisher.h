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


#ifndef GNSSATINFOPUBLISHER_H
#define GNSSATINFOPUBLISHER_H

#include "packetcallback.h"
#include <xsens_mti_ros2_driver/msg/gnss_sat_info.hpp>
#include <xstypes/xsrawgnsssatinfo.h>

struct GnssSatInfoPublisher : public PacketCallback
{
    rclcpp::Publisher<xsens_mti_ros2_driver::msg::GnssSatInfo>::SharedPtr pub;

    GnssSatInfoPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<xsens_mti_ros2_driver::msg::GnssSatInfo>("/gnss/satinfo", pub_queue_size);
    }

    static xsens_mti_ros2_driver::msg::GnssSatInfo buildMessage(const XsRawGnssSatInfo &satinfo)
    {
        xsens_mti_ros2_driver::msg::GnssSatInfo msg;

        msg.itow    = satinfo.m_itow;
        msg.num_svs = satinfo.m_numSvs;
        msg.res1    = satinfo.m_res1;
        msg.res2    = satinfo.m_res2;
        msg.res3    = satinfo.m_res3;

        uint8_t count = satinfo.m_numSvs;
        if (count > 60)
            count = 60;

        for (uint8_t i = 0; i < count; ++i)
        {
            msg.gnss_id[i] = satinfo.m_satInfos[i].m_gnssId;
            msg.sv_id[i]   = satinfo.m_satInfos[i].m_svId;
            msg.cno[i]     = satinfo.m_satInfos[i].m_cno;
            msg.flags[i]   = satinfo.m_satInfos[i].m_flags;
        }

        return msg;
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        (void)timestamp;
        if (packet.containsRawGnssSatInfo())
        {
            XsRawGnssSatInfo satinfo = packet.rawGnssSatInfo();
            pub->publish(buildMessage(satinfo));
        }
    }
};

#endif
