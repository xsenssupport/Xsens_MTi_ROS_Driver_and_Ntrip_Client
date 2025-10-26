#ifndef HIGH_RATE_INTERPOLATOR_H
#define HIGH_RATE_INTERPOLATOR_H

#include <rclcpp/rclcpp.hpp>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsvector.h>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>

struct TimedOrientation
{
    uint32_t sampleTimeFine;
    XsQuaternion quaternion;
};

struct TimedRateOfTurn
{
    uint32_t sampleTimeFine;
    XsVector gyro;
};

struct TimedAcceleration
{
    uint32_t sampleTimeFine;
    XsVector accel;
};

class HighRateInterpolator
{
public:
    HighRateInterpolator(rclcpp::Node::SharedPtr node, size_t bufferSize = 10);
    ~HighRateInterpolator();

    // Process incoming packet and return interpolated packet if ready
    bool processPacket(const XsDataPacket& packet, XsDataPacket& interpolatedPacket);

private:
    // Buffer management
    void addOrientationData(uint32_t sampleTimeFine, const XsQuaternion& quat);
    void addRateOfTurnData(uint32_t sampleTimeFine, const XsVector& gyro);
    void addAccelerationData(uint32_t sampleTimeFine, const XsVector& accel);
    
    // Interpolation methods
    bool interpolateOrientation(uint32_t targetTime, XsQuaternion& result);
    bool interpolateRateOfTurn(uint32_t targetTime, XsVector& result);
    
    // Utility functions
    int64_t computeTimeDifference(uint32_t t1, uint32_t t2);
    double computeInterpolationFactor(uint32_t targetTime, uint32_t t1, uint32_t t2);
    XsQuaternion slerp(const XsQuaternion& q1, const XsQuaternion& q2, double t);
    XsVector lerp(const XsVector& v1, const XsVector& v2, double t);
    
    // Buffer cleanup
    void cleanOldData(uint32_t currentTime);

private:
    rclcpp::Node::SharedPtr m_node;
    size_t m_bufferSize;
    
    std::mutex m_mutex;
    
    std::deque<TimedOrientation> m_orientationBuffer;
    std::deque<TimedRateOfTurn> m_rateOfTurnBuffer;
    std::deque<TimedAcceleration> m_accelerationBuffer;

    // Startup tracking
    bool m_isInitialized;
    size_t m_accelPacketCount;
    static constexpr size_t INITIALIZATION_PACKET_THRESHOLD = 50; // ~0.5s at 100Hz
    
    // Constants for timestamp wrap handling
    static constexpr uint32_t UINT32_MAX_VALUE = 0xFFFFFFFF;
    static constexpr int64_t WRAP_THRESHOLD = 0x7FFFFFFF; // Half of uint32 max
};

#endif // HIGH_RATE_INTERPOLATOR_H