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
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

struct ODOMETRYPublisher : public PacketCallback
{
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    std::string odom_init_frame_id = "odom_init";
    std::string base_frame_id = "base_link"; 

    // Member variables to store initial UTM position and zone
    struct UTMCoordinate
    {
        double easting = 0.0;
        double northing = 0.0;
        double altitude = 0.0;
        int zone = 0;
    } m_utm0;

    // TF broadcasters
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_;

    double m_latitude = 0.0;
    double m_longitude = 0.0;

    ODOMETRYPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;

        node->get_parameter("publisher_queue_size", pub_queue_size);
        node->get_parameter("frame_id", frame_id);

        pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry", pub_queue_size);

        m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
        m_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    /**
     * @brief Initializes the UTM (Universal Transverse Mercator) coordinates based on latitude, longitude, and altitude.
     *
     * This function calculates the appropriate UTM zone for the given geographic coordinates,
     * adjusts for special cases in certain latitude and longitude ranges, converts the latitude
     * and longitude to UTM easting and northing values, and logs the initialized UTM information.
     *
     * @param Lat      Latitude in decimal degrees.
     * @param Long     Longitude in decimal degrees.
     * @param altitude Altitude in meters.
     */
    void initUTM(double Lat, double Long, double altitude)
    {
        int zoneNbr;

        // Normalize longitude to be within the range (-180, 180)
        double normalizedLongitude = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

        // Calculate the initial UTM zone number based on normalized longitude
        zoneNbr = int((normalizedLongitude + 180) / 6) + 1;

        // Adjust zone number for Norway (Latitude between 56°N and 64°N and Longitude between 3°E and 12°E)
        if (Lat >= 56.0 && Lat < 64.0 && normalizedLongitude >= 3.0 && normalizedLongitude < 12.0)
        {
            zoneNbr = 32;
        }

        // Adjust zone number for Svalbard (Latitude between 72°N and 84°N)
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
        // Convert latitude and longitude to UTM easting and northing using the determined zone
        LLtoUTM(Lat, Long, m_utm0.zone, m_utm0.easting, m_utm0.northing);

        // Log the initialized UTM information for debugging or informational purposes
        RCLCPP_INFO(rclcpp::get_logger("ODOMETRYPublisher"), "Initialized UTM Zone %d, Easting: %f, Northing: %f", m_utm0.zone, m_utm0.easting, m_utm0.northing);
    }



    /**
     * @brief Populates and broadcasts a TransformStamped message based on the provided pose and frame information.
     *
     * This function fills in a `geometry_msgs::msg::TransformStamped` message with the given parent and child frame IDs,
     * pose data (position and orientation), and a timestamp. After populating the message, it broadcasts the transform
     * using a TF broadcaster, allowing other nodes in the ROS 2 system to access the transformation between frames.
     *
     * @param parent_frame_id    The identifier of the parent coordinate frame.
     * @param child_frame_id     The identifier of the child coordinate frame.
     * @param pose               The pose data containing position and orientation to define the transform.
     * @param transformStampedMsg Reference to a TransformStamped message that will be populated and sent.
     * @param timestamp          The time at which the transform is valid.
     */
    void fillTransform(
        const std::string &parent_frame_id,
        const std::string &child_frame_id,
        const geometry_msgs::msg::Pose &pose,
        geometry_msgs::msg::TransformStamped &transformStampedMsg,
        rclcpp::Time timestamp)
    {
        // Set the timestamp for the transform message to the provided time.
        transformStampedMsg.header.stamp = timestamp;

        // Assign the parent frame ID to the transform message.
        transformStampedMsg.header.frame_id = parent_frame_id;

        // Assign the child frame ID to the transform message.
        transformStampedMsg.child_frame_id = child_frame_id;

        // Populate the translation component of the transform with the position data from the pose.
        transformStampedMsg.transform.translation.x = pose.position.x;
        transformStampedMsg.transform.translation.y = pose.position.y;
        transformStampedMsg.transform.translation.z = pose.position.z;

        // Populate the rotation component of the transform with the orientation data from the pose.
        transformStampedMsg.transform.rotation = pose.orientation;

        // Broadcast the populated TransformStamped message using the TF broadcaster.
        m_tf_broadcaster_->sendTransform(transformStampedMsg);
    }



    /**
     * @brief Computes the central meridian for a given UTM (Universal Transverse Mercator) zone.
     *
     * This function calculates the central meridian (longitude) of a specified UTM zone.
     * It accounts for standard zones as well as special exceptions for regions like Norway and Svalbard.
     * Additionally, it handles polar regions by allowing zone numbers beyond the standard 60 zones.
     *
     * @param zone_number The UTM zone number for which the central meridian is to be computed.
     *                    - Standard UTM zones range from 1 to 60.
     *                    - Extended zones (61-120) may be used for polar regions or other specialized applications.
     *
     * @return double The longitude of the central meridian in decimal degrees.
     *
     * @throws std::out_of_range If the provided zone number is outside the valid range (1-120).
     */
    double computeMeridian(int zone_number)
    {
        // Special Case: Zone number 0
        // Typically, UTM zones start at 1. Zone 0 might be used as a placeholder or default value.
        if (zone_number == 0)
        {
            return 0.0; // Return 0 degrees longitude for zone 0
        }
        // Error Handling: Invalid Zone Numbers
        // UTM zones are typically numbered from 1 to 60. This function allows up to 120 for extended regions.
        else if (zone_number < 1 || zone_number > 120)
        {
            throw std::out_of_range("Invalid UTM zone number. Must be between 1 and 60 (or 61-120 for polar regions).");
        }
        // Handling Extended Zones (61-120) for Polar Regions
        // These zones may be used in specialized applications or projections for polar areas.
        else if (zone_number >= 61 && zone_number <= 120)
        {
            return 0.0; // Placeholder: Central meridian computation for extended zones can be customized as needed
        }

        // Special Case: Norway's Exception in UTM Zone 32
        // The UTM system has exceptions for regions to minimize distortion. Norway's region between 56°N-64°N and 3°E-12°E
        // is assigned to Zone 32 with a central meridian of 9°E instead of the standard calculation.
        if (zone_number == 32 && m_latitude >= 56.0 && m_latitude < 64.0 && m_longitude >= 3.0 && m_longitude < 12.0)
        {
            return 9.0; // Central meridian for Norway's special case
        }
        // Special Case: Svalbard's Exceptions in UTM Zones 31, 33
        // Svalbard spans multiple UTM zones with adjusted central meridians to account for its extensive east-west range.
        else if (zone_number == 31 && m_latitude >= 72.0 && m_latitude < 84.0)
        {
            return 3.0; // Central meridian for Svalbard's western part
        }
        else if (zone_number == 33 && m_latitude >= 72.0 && m_latitude < 84.0)
        {
            return 33.0; // Central meridian for Svalbard's eastern part
        }

        // Standard Central Meridian Calculation for UTM Zones 1-60
        // The central meridian for a UTM zone is calculated as:
        // Central Meridian = (Zone Number * 6) - 183 degrees
        // Alternatively, it can be expressed as (Zone Number - 1) * 6 - 177 degrees
        // Both formulas yield the same result.
        return (zone_number - 1) * 6.0 - 177.0; // Compute central meridian based on standard UTM zone numbering
    }


  
    /**
     * @brief Converts geographic coordinates (latitude and longitude) to UTM (Universal Transverse Mercator) coordinates.
     *
     * This function implements the conversion from geographic coordinates (latitude and longitude) to UTM easting and northing
     * based on the specified UTM zone number. The conversion accounts for the Earth's ellipsoidal shape using the WGS84
     * reference ellipsoid parameters.
     *
     * @param Lat           Latitude in decimal degrees.
     * @param Long          Longitude in decimal degrees.
     * @param zoneNumber    UTM zone number (1 to 60).
     * @param UTMEasting    Reference to a double where the calculated UTM easting will be stored.
     * @param UTMNorthing   Reference to a double where the calculated UTM northing will be stored.
     */
    void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMEasting, double &UTMNorthing)
    {
        // Define constants used in the conversion based on the WGS84 ellipsoid
        const double pi = M_PI;  
        const double a = 6378137.0;         // Earth's equatorial radius in meters (WGS84)
        const double eccSquared = 0.00669438; // Square of Earth's eccentricity (WGS84)
        const double k0 = 0.9996;           // Scale factor for UTM

        // Normalize longitude to be within the range (-180, 180)
        double normalizedLongitude = (Long + 180.0) - int((Long + 180.0) / 360.0) * 360.0 - 180.0;

        // Convert latitude and longitude from degrees to radians
        double LatRad = Lat * pi / 180.0;
        double LongRad = normalizedLongitude * pi / 180.0;

        // Calculate the longitude of the central meridian for the given UTM zone
        double LongOrigin = (zoneNumber - 1) * 6.0 - 180.0 + 3.0; // +3 shifts to the central meridian of the zone
        double LongOriginRad = LongOrigin * pi / 180.0;

        // Calculate the square of the second eccentricity
        double eccPrimeSquared = eccSquared / (1.0 - eccSquared);

        // Calculate the radius of curvature in the prime vertical
        double N = a / sqrt(1.0 - eccSquared * sin(LatRad) * sin(LatRad));

        // Calculate the square of the tangent of the latitude
        double T = tan(LatRad) * tan(LatRad);

        // Calculate the cosine of the latitude squared, multiplied by the square of the second eccentricity
        double C = eccPrimeSquared * cos(LatRad) * cos(LatRad);

        // Calculate the difference in longitude from the central meridian, in radians
        double A = cos(LatRad) * (LongRad - LongOriginRad);

        // Calculate the meridional arc length
        double M = a * ((1.0 - eccSquared / 4.0 - 3.0 * eccSquared * eccSquared / 64.0 - 5.0 * pow(eccSquared, 3) / 256.0) * LatRad
                    - (3.0 * eccSquared / 8.0 + 3.0 * eccSquared * eccSquared / 32.0 + 45.0 * pow(eccSquared, 3) / 1024.0) * sin(2.0 * LatRad)
                    + (15.0 * eccSquared * eccSquared / 256.0 + 45.0 * pow(eccSquared, 3) / 1024.0) * sin(4.0 * LatRad)
                    - (35.0 * pow(eccSquared, 3) / 3072.0) * sin(6.0 * LatRad));

        // Calculate the UTM Easting (x-coordinate)
        UTMEasting = k0 * N * (A + (1.0 - T + C) * pow(A, 3) / 6.0
                                + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * eccPrimeSquared) * pow(A, 5) / 120.0)
                    + 500000.0; // Add false easting

        // Calculate the UTM Northing (y-coordinate)
        UTMNorthing = k0 * (M + N * tan(LatRad) * (A * A / 2.0
                            + (5.0 - T + 9.0 * C + 4.0 * C * C) * pow(A, 4) / 24.0
                            + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * eccPrimeSquared) * pow(A, 6) / 720.0));

        // If the latitude is in the southern hemisphere, add 10,000,000 meters to the northing
        if (Lat < 0.0)
        {
            UTMNorthing += 10000000.0; // Add false northing for southern hemisphere
        }
    }



    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        if (packet.containsPositionLLA() && packet.containsOrientation() && packet.containsCalibratedGyroscopeData() && packet.containsVelocity())
        {
            XsVector p = packet.positionLLA(); // p[0]: lat, p[1]: lon, p[2]: alt in degrees and meters
            XsVector v = packet.velocity();    // v[0], v[1], v[2]: velocity in x, y, z (m/s)
            XsVector gyro = packet.calibratedGyroscopeData(); // gyro[0], gyro[1], gyro[2]: angular rates in rad/s
            XsQuaternion q = packet.orientationQuaternion();  // Orientation quaternion

            nav_msgs::msg::Odometry msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;
            msg.child_frame_id = base_frame_id;

            // Set orientation
            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();

            // Convert latitude and longitude to UTM coordinates
            double utm_easting, utm_northing;
            LLtoUTM(p[0], p[1], m_utm0.zone, utm_easting, utm_northing);

            // Initialize initial UTM position if necessary
            if (m_utm0.zone == 0)
            {
                initUTM(p[0], p[1], p[2]);

                // Publish UTM initial transformation
                geometry_msgs::msg::Pose pose;
                pose.position.x = m_utm0.easting;
                pose.position.y = m_utm0.northing;
                pose.position.z = m_utm0.altitude;

                // Set orientation to identity (no rotation)
                pose.orientation.w = 1.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;

                geometry_msgs::msg::TransformStamped transform;
                fillTransform(odom_init_frame_id, frame_id, pose, transform, timestamp);
                m_static_tf_broadcaster_->sendTransform(transform);
            }

            // Compute position relative to initial position
            msg.pose.pose.position.x = utm_easting - m_utm0.easting;
            msg.pose.pose.position.y = utm_northing - m_utm0.northing;
            msg.pose.pose.position.z = p[2] - m_utm0.altitude;

            // Compute convergence angle
            double latitudeRad = p[0] * M_PI / 180.0;
            double longitudeRad = p[1] * M_PI / 180.0;
            double central_meridian_deg = computeMeridian(m_utm0.zone);
            double central_meridian = central_meridian_deg * M_PI / 180.0;
            double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));

            // Set linear velocities
            msg.twist.twist.linear.x = v[0];
            msg.twist.twist.linear.y = v[1];
            msg.twist.twist.linear.z = v[2];

            // Set angular velocities
            msg.twist.twist.angular.x = gyro[0];
            msg.twist.twist.angular.y = gyro[1];
            msg.twist.twist.angular.z = gyro[2];

            // Publish the odometry message
            pub->publish(msg);

            // Publish odometry transformation
            geometry_msgs::msg::Pose pose;
            pose.position = msg.pose.pose.position;
            pose.orientation = msg.pose.pose.orientation;

            geometry_msgs::msg::TransformStamped transform;
            fillTransform(msg.header.frame_id, msg.child_frame_id, pose, transform, timestamp);
            m_tf_broadcaster_->sendTransform(transform);
        }
    }
};

#endif
