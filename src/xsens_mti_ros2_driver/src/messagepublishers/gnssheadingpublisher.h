//  Copyright (c) 2003-2026 Movella Technologies B.V. or subsidiaries worldwide.
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


#ifndef GNSSHEADINGPUBLISHER_H
#define GNSSHEADINGPUBLISHER_H

#include "packetcallback.h"
#include <xsens_mti_ros2_driver/msg/gnss_heading.hpp>
#include <xstypes/xsrawgnssheading.h>

struct GnssHeadingPublisher : public PacketCallback
{
    DriverPublisher<xsens_mti_ros2_driver::msg::GnssHeading> pub;
    std::string frame_id = "world";

    GnssHeadingPublisher(DriverNode::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<xsens_mti_ros2_driver::msg::GnssHeading>("/gnss/heading", pub_queue_size);
        node->get_parameter("fixed_frame_id", frame_id);
    }

    static xsens_mti_ros2_driver::msg::GnssHeading buildMessage(
        const XsRawGnssHeading &heading,
        rclcpp::Time timestamp,
        const std::string &frame_id)
    {
        constexpr double MM_TO_M = 1e-3;
        constexpr double DEG_1E5_TO_DEG = 1e-5;

        xsens_mti_ros2_driver::msg::GnssHeading msg;

        msg.header.stamp    = timestamp;
        msg.header.frame_id = frame_id;

        msg.i_tow = heading.m_itow;

        msg.rel_pos_n       = heading.m_relPosN * MM_TO_M;
        msg.rel_pos_e       = heading.m_relPosE * MM_TO_M;
        msg.rel_pos_d       = heading.m_relPosD * MM_TO_M;
        msg.rel_pos_length  = heading.m_relPosLength * MM_TO_M;
        msg.rel_pos_heading = heading.m_relPosHeading * DEG_1E5_TO_DEG;

        msg.acc_n       = heading.m_accN * MM_TO_M;
        msg.acc_e       = heading.m_accE * MM_TO_M;
        msg.acc_d       = heading.m_accD * MM_TO_M;
        msg.acc_length  = heading.m_accLength * MM_TO_M;
        msg.acc_heading = heading.m_accHeading * DEG_1E5_TO_DEG;

        msg.flags                    = heading.m_flags;
        msg.gnss_fix_ok              = (heading.m_flags & XGHF_GnssFixOk) != 0;
        msg.diff_corrections_applied = (heading.m_flags & XGHF_DiffCorrectionsApplied) != 0;
        msg.rel_pos_valid            = (heading.m_flags & XGHF_RelativePositionValid) != 0;
        msg.heading_valid            = (heading.m_flags & XGHF_RelativeHeadingValid) != 0;
        msg.carrier_solution         = static_cast<uint8_t>((heading.m_flags & XGHF_CarrSolnMask) >> XGHFO_CarrSoln);

        return msg;
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsRawGnssHeading())
        {
            XsRawGnssHeading heading = packet.rawGnssHeading();
            pub->publish(buildMessage(heading, timestamp, frame_id));
        }
    }
};

#endif
