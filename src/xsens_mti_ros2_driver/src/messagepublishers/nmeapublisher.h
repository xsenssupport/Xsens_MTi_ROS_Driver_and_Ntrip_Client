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

#ifndef NEMAPUBLISHER_H
#define NEMAPUBLISHER_H

#include "packetcallback.h"
#include <nmea_msgs/msg/sentence.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "ntrip_util.h"

struct NMEAPublisher : public PacketCallback
{
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    rclcpp::Logger logger; // Store the logger

    XsDataPacket latest_packet; // to store the latest packet
    bool new_data_available = false;
    int packet_counter = 0; // Counter to track the number of packets received

    NMEAPublisher(rclcpp::Node::SharedPtr node)
    : logger(node->get_logger()) // Initialize the logger
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        node->get_parameter("frame_id", frame_id);

        pub = node->create_publisher<nmea_msgs::msg::Sentence>("/nmea", pub_queue_size);

    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        nmea_msgs::msg::Sentence nmea_msg;
        nmea_msg.header.stamp = timestamp;
        nmea_msg.header.frame_id = frame_id;

        if (packet.containsRawGnssPvtData() && packet.containsStatus())
        {
            std::string gga_buffer;
            int result = libntrip::generateGGA(packet, &gga_buffer);
            if (result == 0) // Check if generateGGA was successful and there's no checksum error
            {
                nmea_msg.sentence = gga_buffer;
                pub->publish(nmea_msg);
            }
            else
            {
                // Optionally, log an error or take other actions if generateGGA fails
                RCLCPP_WARN(logger, "Failed to generate valid GPGGA message. Checksum error detected.");
            }
        }
        
    }


};

#endif

