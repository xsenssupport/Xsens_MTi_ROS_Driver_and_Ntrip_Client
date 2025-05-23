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


#ifndef IMUHRPUBLISHER_H
#define IMUHRPUBLISHER_H

#include <optional>

#include "packetcallback.h"
#include "publisherhelperfunctions.h"
#include <sensor_msgs/msg/imu.hpp>
// basic file operations
#include <iostream>
#include <fstream>

// enable for debugging save to file
// #define IMU_HR_DEBUG

struct ImuHRPublisher : public PacketCallback, PublisherHelperFunctions
{
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    double orientation_variance[3];
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];
    rclcpp::Node::SharedPtr node_handle;
    // Varaiables to store the last received unpublished value
    std::optional<geometry_msgs::msg::Vector3> last_accel;
    std::optional<geometry_msgs::msg::Vector3> last_gyro;
    uint64_t last_acc_timestamp = 0;
    uint64_t last_gyr_timestamp = 0;

    std::ofstream debug_file;

    std::string frame_id = DEFAULT_FRAME_ID;
    uint64_t acc_gyr_timestamp_diff_threshold = s2ns(0.0005); // 0.5ms

    ImuHRPublisher(rclcpp::Node::SharedPtr node)
        : node_handle(node)
    {
        std::vector<double> variance = {0, 0, 0};
        node->declare_parameter("orientation_stddev", variance);
        node->declare_parameter("angular_velocity_stddev", variance);
        node->declare_parameter("linear_acceleration_stddev", variance);

        int pub_queue_size = 100;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.keep_last(pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);

        // REP 145: Conventions for IMU Sensor Drivers (http://www.ros.org/reps/rep-0145.html)
        variance_from_stddev_param("orientation_stddev", orientation_variance, node);
        variance_from_stddev_param("angular_velocity_stddev", angular_velocity_variance, node);
        variance_from_stddev_param("linear_acceleration_stddev", linear_acceleration_variance, node);

        node_handle->get_parameter("frame_id", frame_id);

        #ifdef IMU_HR_DEBUG
        debug_file.open ("/home/radius/imu_hr_publisher_output.txt");
        #endif
    }

    /////////////////////////////////////////////////////////////////
    // taken from vilens_slam_utils::time.hpp
    inline uint64_t s2ns(const double t) {
        if (t < 0) {
            throw std::runtime_error("Trying to convert negative number to uint");
        }
        return static_cast<uint64_t>(t * 1'000'000'000);
    }
    inline double ns2s(const uint64_t t) {
        return static_cast<double>(t) / 1'000'000'000;
    }
    inline void splitNs(const uint64_t t, uint64_t &s, uint64_t &ns) {
        s = static_cast<uint64_t>(std::floor(ns2s(t)));
        ns = t - s2ns(s);
        }

    inline std::string ns2string(const uint64_t t) {
        uint64_t s, ns;
        splitNs(t, s, ns);
        std::stringstream output;
        output << s << "." << std::setw(9) << std::setfill('0') << ns;
        return output.str();
    }
    /////////////////////////////////////////////////////////////////

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        bool quaternion_available = packet.containsOrientation();
        bool gyro_hr_available = packet.containsRateOfTurnHR();
        bool accel_hr_available = packet.containsAccelerationHR();

        geometry_msgs::msg::Quaternion quaternion;
        if (quaternion_available)
        {
            XsQuaternion q = packet.orientationQuaternion();

            quaternion.w = q.w();
            quaternion.x = q.x();
            quaternion.y = q.y();
            quaternion.z = q.z();
        }

        geometry_msgs::msg::Vector3 gyro;
        if (gyro_hr_available)
        {
            XsVector g = packet.rateOfTurnHR();
            gyro.x = g[0];
            gyro.y = g[1];
            gyro.z = g[2];
            last_gyro = gyro;
            uint64_t curr_timestamp = timestamp.nanoseconds();
            #ifdef IMU_HR_DEBUG
            debug_file << ns2string(curr_timestamp) << ": gyr\n";
            if (curr_timestamp - last_gyr_timestamp > 3e6) {
                debug_file << "!!! missing gyr measurements. delta-t: " 
                << ns2string(curr_timestamp - last_gyr_timestamp) << "\n";
            }
            last_gyr_timestamp = curr_timestamp;
            #endif
        }

        geometry_msgs::msg::Vector3 accel;
        if (accel_hr_available)
        {
            XsVector a = packet.accelerationHR();
            accel.x = a[0];
            accel.y = a[1];
            accel.z = a[2];
            last_accel = accel;
            uint64_t curr_timestamp = timestamp.nanoseconds();
            #ifdef IMU_HR_DEBUG
            debug_file << ns2string(curr_timestamp) << ": acc\n";
            if (curr_timestamp - last_acc_timestamp > 3e6) {
                debug_file << "!!! missing acc measurements. delta-t: " 
                << ns2string(curr_timestamp - last_acc_timestamp) << "\n";
            }
            last_acc_timestamp = curr_timestamp;
            #endif
        }

        // Imu message, publish if any of the fields is available
        if (!last_accel.has_value() || !last_gyro.has_value()) {
            #ifdef IMU_HR_DEBUG
            debug_file << "skipping - only one meas available\n";
            #endif
            return;
        }

        // force the algorithm to only publish correct pairs of timestamps
        uint64_t acc_gyr_diff = last_gyr_timestamp - last_acc_timestamp;
        if (last_acc_timestamp > last_gyr_timestamp) {
            acc_gyr_diff = last_acc_timestamp - last_gyr_timestamp;
        }
        
        if (acc_gyr_diff > acc_gyr_timestamp_diff_threshold) {
            #ifdef IMU_HR_DEBUG
            debug_file << "skipping - not publishing points with incorrect timestamp diff: " << ns2string(acc_gyr_diff) << "s\n";
            #endif
            return;
        }

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        msg.orientation = quaternion;
        if (quaternion_available)
        {
            msg.orientation_covariance[0] = orientation_variance[0];
            msg.orientation_covariance[4] = orientation_variance[1];
            msg.orientation_covariance[8] = orientation_variance[2];
        }
        else
        {
            msg.orientation_covariance[0] = -1; // mark as not available
        }

        msg.angular_velocity = *last_gyro;
        if (gyro_hr_available)
        {
            msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
            msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
            msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
        }
        else
        {
            msg.angular_velocity_covariance[0] = -1; // mark as not available
        }

        msg.linear_acceleration = *last_accel;
        if (accel_hr_available)
        {
            msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
            msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
            msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
        }
        else
        {
            msg.linear_acceleration_covariance[0] = -1; // mark as not available
        }

        #ifdef IMU_HR_DEBUG
        debug_file << "publishing\n\n";
        #endif
        pub->publish(msg);
        
        // Removing published values
        last_accel.reset();
        last_gyro.reset();
    }
};

#endif
