#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsvector.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsdataidentifier.h>

#include "messagepublishers/imupublisher.h"
#include "messagepublishers/gnsspvtpublisher.h"
#include <xstypes/xsrawgnsspvtdata.h>

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

// ── GnssPvtPublisher::buildMessage ───────────────────────────────────────────

static XsRawGnssPvtData makePvt()
{
    XsRawGnssPvtData pvt;
    memset(&pvt, 0, sizeof(pvt));
    pvt.m_itow    = 123456789;
    pvt.m_year    = 2024;
    pvt.m_month   = 4;
    pvt.m_day     = 7;
    pvt.m_hour    = 12;
    pvt.m_min     = 30;
    pvt.m_sec     = 45;
    pvt.m_valid   = 0x07;
    pvt.m_tAcc    = 50;
    pvt.m_nano    = 123456;
    pvt.m_fixType = 3;   // 3D fix
    pvt.m_flags   = 0x01; // gnssFixOk
    pvt.m_numSv   = 12;
    pvt.m_lon     = 1138523400; // ~113.85234 deg
    pvt.m_lat     =  226543200; //  ~22.65432 deg
    pvt.m_height  = 50000;      // 50 m
    pvt.m_hMsl    = 48000;
    pvt.m_hAcc    = 1500;
    pvt.m_vAcc    = 2000;
    pvt.m_velN    = 100;
    pvt.m_velE    = 200;
    pvt.m_velD    = -10;
    pvt.m_gSpeed  = 224;
    pvt.m_headMot = 6300000; // 63 deg
    pvt.m_sAcc    = 500;
    pvt.m_headAcc = 200000;
    pvt.m_headVeh = 6200000;
    pvt.m_pdop    = 120;
    return pvt;
}

TEST(GnssPvtPublisherBuildMessage, FieldsAreMappedCorrectly)
{
    XsRawGnssPvtData pvt = makePvt();
    auto msg = GnssPvtPublisher::buildMessage(pvt, rclcpp::Time(0), "world");

    EXPECT_EQ(msg.i_tow,    pvt.m_itow);
    EXPECT_EQ(msg.year,     pvt.m_year);
    EXPECT_EQ(msg.month,    pvt.m_month);
    EXPECT_EQ(msg.day,      pvt.m_day);
    EXPECT_EQ(msg.hour,     pvt.m_hour);
    EXPECT_EQ(msg.min,      pvt.m_min);
    EXPECT_EQ(msg.sec,      pvt.m_sec);
    EXPECT_EQ(msg.valid,    pvt.m_valid);
    EXPECT_EQ(msg.t_acc,    pvt.m_tAcc);
    EXPECT_EQ(msg.nano,     pvt.m_nano);
    EXPECT_EQ(msg.fix_type, pvt.m_fixType);
    EXPECT_EQ(msg.flags,    pvt.m_flags);
    EXPECT_EQ(msg.num_sv,   pvt.m_numSv);
    EXPECT_EQ(msg.lon,      pvt.m_lon);
    EXPECT_EQ(msg.lat,      pvt.m_lat);
    EXPECT_EQ(msg.height,   pvt.m_height);
    EXPECT_EQ(msg.h_msl,    pvt.m_hMsl);
    EXPECT_EQ(msg.h_acc,    pvt.m_hAcc);
    EXPECT_EQ(msg.v_acc,    pvt.m_vAcc);
    EXPECT_EQ(msg.vel_n,    pvt.m_velN);
    EXPECT_EQ(msg.vel_e,    pvt.m_velE);
    EXPECT_EQ(msg.vel_d,    pvt.m_velD);
    EXPECT_EQ(msg.g_speed,  pvt.m_gSpeed);
    EXPECT_EQ(msg.heading,  pvt.m_headMot);
    EXPECT_EQ(msg.s_acc,    pvt.m_sAcc);
    EXPECT_EQ(msg.head_acc, pvt.m_headAcc);
    EXPECT_EQ(msg.head_veh, pvt.m_headVeh);
    EXPECT_EQ(msg.p_dop,    pvt.m_pdop);
}

TEST(GnssPvtPublisherBuildMessage, HeaderIsSet)
{
    XsRawGnssPvtData pvt = makePvt();
    rclcpp::Time stamp(42, 0);
    auto msg = GnssPvtPublisher::buildMessage(pvt, stamp, "world");

    EXPECT_EQ(msg.header.frame_id, "world");
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
