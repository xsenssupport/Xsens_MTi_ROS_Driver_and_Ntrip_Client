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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

struct ODOMETRYPublisher : public PacketCallback
{
    ros::Publisher pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    std::string odom_init_frame_id = "odom_init";
    std::string base_frame_id = "base_link";

    struct UTMCoordinate
    {
        double easting = 0.0;
        double northing = 0.0;
        double altitude = 0.0;
        int zone = 0;
    } m_utm0;

    tf2_ros::StaticTransformBroadcaster m_static_tf_broadcaster;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    double m_latitude = 0.0;
    double m_longitude = 0.0;

    ODOMETRYPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        ros::param::get("~frame_id", frame_id);
        pub = node.advertise<nav_msgs::Odometry>("/odometry", pub_queue_size);
    }

    void initUTM(double Lat, double Long, double altitude)
    {
        int zoneNbr;
        double normalizedLongitude = (Long + 180) - int((Long + 180) / 360) * 360 - 180;
        zoneNbr = int((normalizedLongitude + 180) / 6) + 1;

        if (Lat >= 56.0 && Lat < 64.0 && normalizedLongitude >= 3.0 && normalizedLongitude < 12.0)
        {
            zoneNbr = 32;
        }

        if (Lat >= 72.0 && Lat < 84.0)
        {
            if (normalizedLongitude >= 0.0 && normalizedLongitude < 9.0)
                zoneNbr = 31;
            else if (normalizedLongitude >= 9.0 && normalizedLongitude < 21.0)
                zoneNbr = 33;
            else if (normalizedLongitude >= 21.0 && normalizedLongitude < 33.0)
                zoneNbr = 35;
            else if (normalizedLongitude >= 33.0 && normalizedLongitude < 42.0)
                zoneNbr = 37;
        }

        m_utm0.zone = zoneNbr;
        m_utm0.altitude = altitude;
        LLtoUTM(Lat, Long, m_utm0.zone, m_utm0.easting, m_utm0.northing);

        ROS_INFO("Initialized UTM Zone %d, Easting: %f, Northing: %f", m_utm0.zone, m_utm0.easting, m_utm0.northing);
    }

    void fillTransform(
        const std::string &parent_frame_id,
        const std::string &child_frame_id,
        const geometry_msgs::Pose &pose,
        geometry_msgs::TransformStamped &transformStampedMsg,
        ros::Time timestamp)
    {
        transformStampedMsg.header.stamp = timestamp;
        transformStampedMsg.header.frame_id = parent_frame_id;
        transformStampedMsg.child_frame_id = child_frame_id;
        transformStampedMsg.transform.translation.x = pose.position.x;
        transformStampedMsg.transform.translation.y = pose.position.y;
        transformStampedMsg.transform.translation.z = pose.position.z;
        transformStampedMsg.transform.rotation = pose.orientation;
        m_tf_broadcaster.sendTransform(transformStampedMsg);
    }

    double computeMeridian(int zone_number)
    {
        if (zone_number == 0)
        {
            return 0.0;
        }
        else if (zone_number < 1 || zone_number > 120)
        {
            throw std::out_of_range("Invalid UTM zone number. Must be between 1 and 60 (or 61-120 for polar regions).");
        }
        else if (zone_number >= 61 && zone_number <= 120)
        {
            return 0.0;
        }

        if (zone_number == 32 && m_latitude >= 56.0 && m_latitude < 64.0 && m_longitude >= 3.0 && m_longitude < 12.0)
        {
            return 9.0;
        }
        else if (zone_number == 31 && m_latitude >= 72.0 && m_latitude < 84.0)
        {
            return 3.0;
        }
        else if (zone_number == 33 && m_latitude >= 72.0 && m_latitude < 84.0)
        {
            return 33.0;
        }

        return (zone_number - 1) * 6.0 - 177.0;
    }

    void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMEasting, double &UTMNorthing)
    {
        const double pi = M_PI;
        const double a = 6378137.0;
        const double eccSquared = 0.00669438;
        const double k0 = 0.9996;

        double normalizedLongitude = (Long + 180.0) - int((Long + 180.0) / 360.0) * 360.0 - 180.0;

        double LatRad = Lat * pi / 180.0;
        double LongRad = normalizedLongitude * pi / 180.0;

        double LongOrigin = (zoneNumber - 1) * 6.0 - 180.0 + 3.0;
        double LongOriginRad = LongOrigin * pi / 180.0;

        double eccPrimeSquared = eccSquared / (1.0 - eccSquared);

        double N = a / sqrt(1.0 - eccSquared * sin(LatRad) * sin(LatRad));
        double T = tan(LatRad) * tan(LatRad);
        double C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
        double A = cos(LatRad) * (LongRad - LongOriginRad);

        double M = a * ((1.0 - eccSquared / 4.0 - 3.0 * eccSquared * eccSquared / 64.0 - 5.0 * pow(eccSquared, 3) / 256.0) * LatRad
                    - (3.0 * eccSquared / 8.0 + 3.0 * eccSquared * eccSquared / 32.0 + 45.0 * pow(eccSquared, 3) / 1024.0) * sin(2.0 * LatRad)
                    + (15.0 * eccSquared * eccSquared / 256.0 + 45.0 * pow(eccSquared, 3) / 1024.0) * sin(4.0 * LatRad)
                    - (35.0 * pow(eccSquared, 3) / 3072.0) * sin(6.0 * LatRad));

        UTMEasting = k0 * N * (A + (1.0 - T + C) * pow(A, 3) / 6.0
                                + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * eccPrimeSquared) * pow(A, 5) / 120.0)
                    + 500000.0;

        UTMNorthing = k0 * (M + N * tan(LatRad) * (A * A / 2.0
                            + (5.0 - T + 9.0 * C + 4.0 * C * C) * pow(A, 4) / 24.0
                            + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * eccPrimeSquared) * pow(A, 6) / 720.0));

        if (Lat < 0.0)
        {
            UTMNorthing += 10000000.0;
        }
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsPositionLLA() && packet.containsOrientation() && packet.containsCalibratedGyroscopeData() && packet.containsVelocity())
        {
            XsVector p = packet.positionLLA();
            XsVector v = packet.velocity();
            XsVector gyro = packet.calibratedGyroscopeData();
            XsQuaternion q = packet.orientationQuaternion();

            nav_msgs::Odometry msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;
            msg.child_frame_id = base_frame_id;

            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();

            double utm_easting, utm_northing;
            LLtoUTM(p[0], p[1], m_utm0.zone, utm_easting, utm_northing);

            if (m_utm0.zone == 0)
            {
                initUTM(p[0], p[1], p[2]);

                geometry_msgs::Pose pose;
                pose.position.x = m_utm0.easting;
                pose.position.y = m_utm0.northing;
                pose.position.z = m_utm0.altitude;

                pose.orientation.w = 1.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;

                geometry_msgs::TransformStamped transform;
                transform.header.stamp = timestamp;
                transform.header.frame_id = odom_init_frame_id;
                transform.child_frame_id = frame_id;
                transform.transform.translation.x = pose.position.x;
                transform.transform.translation.y = pose.position.y;
                transform.transform.translation.z = pose.position.z;
                transform.transform.rotation = pose.orientation;
                m_static_tf_broadcaster.sendTransform(transform);
            }

            msg.pose.pose.position.x = utm_easting - m_utm0.easting;
            msg.pose.pose.position.y = utm_northing - m_utm0.northing;
            msg.pose.pose.position.z = p[2] - m_utm0.altitude;

            double latitudeRad = p[0] * M_PI / 180.0;
            double longitudeRad = p[1] * M_PI / 180.0;
            double central_meridian_deg = computeMeridian(m_utm0.zone);
            double central_meridian = central_meridian_deg * M_PI / 180.0;
            double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));

            msg.twist.twist.linear.x = v[0];
            msg.twist.twist.linear.y = v[1];
            msg.twist.twist.linear.z = v[2];

            msg.twist.twist.angular.x = gyro[0];
            msg.twist.twist.angular.y = gyro[1];
            msg.twist.twist.angular.z = gyro[2];

            pub.publish(msg);

            geometry_msgs::Pose pose;
            pose.position = msg.pose.pose.position;
            pose.orientation = msg.pose.pose.orientation;

            geometry_msgs::TransformStamped transform;
            fillTransform(msg.header.frame_id, msg.child_frame_id, pose, transform, timestamp);
            m_tf_broadcaster.sendTransform(transform);
        }
    }
};

#endif
