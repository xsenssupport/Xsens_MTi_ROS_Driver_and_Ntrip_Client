#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsvector.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsdataidentifier.h>

#include "messagepublishers/imupublisher.h"

// Default variances used across tests
static const double kOrientationVar[3]        = {0.01, 0.02, 0.03};
static const double kAngularVelocityVar[3]    = {0.001, 0.002, 0.003};
static const double kLinearAccelerationVar[3] = {0.1, 0.2, 0.3};

// ── Pure-IMU device (e.g. MTi-610): accel + gyro, no orientation ─────────────

TEST(ImuPublisherBuildMessage, PureImu_OrientationCovarianceIsMinusOne)
{
    XsDataPacket packet;
    XsReal accel_data[3] = {0.1, 0.2, 9.8};
    packet.setCalibratedAcceleration(XsVector(3, accel_data));
    XsReal gyro_data[3] = {0.01, 0.02, 0.03};
    packet.setCalibratedGyroscopeData(XsVector(3, gyro_data));

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0)
        << "orientation_covariance[0] must be -1 when orientation is unavailable (REP-145)";
}

TEST(ImuPublisherBuildMessage, PureImu_AccelAndGyroValuesCorrect)
{
    XsDataPacket packet;
    XsReal accel_data[3] = {1.0, 2.0, 3.0};
    packet.setCalibratedAcceleration(XsVector(3, accel_data));
    XsReal gyro_data[3] = {0.1, 0.2, 0.3};
    packet.setCalibratedGyroscopeData(XsVector(3, gyro_data));

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, 3.0);

    EXPECT_DOUBLE_EQ(msg.angular_velocity.x, 0.1);
    EXPECT_DOUBLE_EQ(msg.angular_velocity.y, 0.2);
    EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.3);
}

TEST(ImuPublisherBuildMessage, PureImu_AccelCovarianceSet)
{
    XsDataPacket packet;
    XsReal accel_data[3] = {0.0, 0.0, 9.81};
    packet.setCalibratedAcceleration(XsVector(3, accel_data));

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[0], kLinearAccelerationVar[0]);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[4], kLinearAccelerationVar[1]);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[8], kLinearAccelerationVar[2]);
}

TEST(ImuPublisherBuildMessage, PureImu_GyroCovarianceMinusOneWhenAbsent)
{
    XsDataPacket packet;
    XsReal accel_data[3] = {0.0, 0.0, 9.81};
    packet.setCalibratedAcceleration(XsVector(3, accel_data));
    // no gyro data

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[0], -1.0)
        << "angular_velocity_covariance[0] must be -1 when gyro data is unavailable";
}

// ── VRU/AHRS device: has orientation output ───────────────────────────────────

TEST(ImuPublisherBuildMessage, Ahrs_OrientationCovarianceSetFromVariance)
{
    XsDataPacket packet;
    packet.setOrientationQuaternion(XsQuaternion(1.0, 0.0, 0.0, 0.0), XDI_CoordSysEnu);

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], kOrientationVar[0]);
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[4], kOrientationVar[1]);
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[8], kOrientationVar[2]);
}

TEST(ImuPublisherBuildMessage, Ahrs_OrientationValuesCorrect)
{
    XsDataPacket packet;
    // Identity quaternion: w=1, x=0, y=0, z=0
    packet.setOrientationQuaternion(XsQuaternion(1.0, 0.0, 0.0, 0.0), XDI_CoordSysEnu);

    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, rclcpp::Time(0), "imu_link",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_DOUBLE_EQ(msg.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(msg.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.orientation.z, 0.0);
}

// ── Header fields ─────────────────────────────────────────────────────────────

TEST(ImuPublisherBuildMessage, HeaderFrameIdAndStampAreSet)
{
    XsDataPacket packet;
    XsReal accel_data[3] = {0.0, 0.0, 9.81};
    packet.setCalibratedAcceleration(XsVector(3, accel_data));

    rclcpp::Time stamp(123, 456);
    sensor_msgs::msg::Imu msg = ImuPublisher::buildImuMessage(
        packet, stamp, "base_imu",
        kOrientationVar, kAngularVelocityVar, kLinearAccelerationVar);

    EXPECT_EQ(msg.header.frame_id, "base_imu");
    EXPECT_EQ(msg.header.stamp, stamp);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
