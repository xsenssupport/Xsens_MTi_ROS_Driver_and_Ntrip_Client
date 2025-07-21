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


#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "packetcallback.h"
#include "xsens_mti_ros2_driver/msg/xs_status_word.hpp"

struct StatusPublisher : public PacketCallback
{
    rclcpp::Publisher<xsens_mti_ros2_driver::msg::XsStatusWord>::SharedPtr pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
    uint32_t last_mgbe_status_ = -1;
    //    diagnostic_updater::Updater updater;
    //std::string frame_id = DEFAULT_FRAME_ID;

    StatusPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;

        node->get_parameter("publisher_queue_size", pub_queue_size);
        //node->get_parameter("frame_id", frame_id);

        pub = node->create_publisher<xsens_mti_ros2_driver::msg::XsStatusWord>("/status", pub_queue_size);
        diag_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    }

    void parseToMessage(xsens_mti_ros2_driver::msg::XsStatusWord &msg, uint32_t status)
    {
        uint32_t temp; // Temporary variable to hold bitwise results

        temp = status & (1 << 0);
        msg.selftest = (temp != 0);

        temp = status & (1 << 1);
        msg.filter_valid = (temp != 0);

        temp = status & (1 << 2);
        msg.gnss_fix = (temp != 0);

        msg.no_rotation_update_status = (status >> 3) & 0x3; // status & 0x18;

        temp = status & (1 << 5);
        msg.representative_motion = (temp != 0);

        temp = status & (1 << 6);
        msg.clock_bias_estimation = (temp != 0);

        temp = status & (1 << 8);
        msg.clipflag_acc_x = (temp != 0);

        temp = status & (1 << 9);
        msg.clipflag_acc_y = (temp != 0);

        temp = status & (1 << 10);
        msg.clipflag_acc_z = (temp != 0);

        temp = status & (1 << 11);
        msg.clipflag_gyr_x = (temp != 0);

        temp = status & (1 << 12);
        msg.clipflag_gyr_y = (temp != 0);

        temp = status & (1 << 13);
        msg.clipflag_gyr_z = (temp != 0);

        temp = status & (1 << 14);
        msg.clipflag_mag_x = (temp != 0);

        temp = status & (1 << 15);
        msg.clipflag_mag_y = (temp != 0);

        temp = status & (1 << 16);
        msg.clipflag_mag_z = (temp != 0);

        temp = status & (1 << 19);
        msg.clipping_indication = (temp != 0);

        temp = status & (1 << 21);
        msg.syncin_marker = (temp != 0);

        temp = status & (1 << 22);
        msg.syncout_marker = (temp != 0);

        msg.filter_mode = (status >> 23) & 0x7; // status & 0x03800000;

        temp = status & (1 << 26);
        msg.have_gnss_time_pulse = (temp != 0);

        msg.rtk_status = (status >> 27) & 0x3; // status & 0x18000000;
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsStatus())
        {
            xsens_mti_ros2_driver::msg::XsStatusWord msg;

            uint32_t status = packet.status();
            parseToMessage(msg, status);

            pub->publish(msg);

            if (status != last_mgbe_status_)
            {
                diagnostic_msgs::msg::DiagnosticArray diag_msg;
                diag_msg.header.stamp = timestamp;

                diagnostic_msgs::msg::DiagnosticStatus robot_status;
                robot_status.name = "IMU status";
                robot_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                robot_status.message = "";

                std::vector<diagnostic_msgs::msg::KeyValue> key_values(1);
                key_values[0].set__key("Manual Gyro Bias Estimation status").set__value(std::to_string(msg.no_rotation_update_status));

                robot_status.values = key_values;
                diag_msg.status.push_back(robot_status);
                diag_pub->publish(diag_msg);

                last_mgbe_status_ = status;
            }
        }
    }
};

#endif
