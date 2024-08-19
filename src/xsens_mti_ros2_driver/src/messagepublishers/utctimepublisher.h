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

#ifndef UTCTIMEPUBLISHER_H
#define UTCTIMEPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/msg/time_reference.hpp>
#include <ctime>

struct UTCTimePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub;

    UTCTimePublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::TimeReference>("imu/utctime", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsUtcTime())
        {
            sensor_msgs::msg::TimeReference msg;

            XsTimeInfo utcTime = packet.utcTime();

            struct tm timeinfo = {0};

            timeinfo.tm_year = utcTime.m_year - 1900; // Years since 1900
            timeinfo.tm_mon = utcTime.m_month - 1;    // Months since January
            timeinfo.tm_mday = utcTime.m_day;
            timeinfo.tm_hour = utcTime.m_hour;
            timeinfo.tm_min = utcTime.m_minute;
            timeinfo.tm_sec = utcTime.m_second;

            // Convert to time_t (seconds since 1st Jan 1970)
            time_t epochSeconds = timegm(&timeinfo);

            // Convert to rclcpp::Time
            rclcpp::Time rosUtcTime(epochSeconds, utcTime.m_nano);

            msg.header.stamp = timestamp;
            msg.time_ref = rosUtcTime;

            pub->publish(msg);
        }
    }
};

#endif
