#include "high_rate_interpolator.h"
#include <cmath>
#include <algorithm>

HighRateInterpolator::HighRateInterpolator(size_t bufferSize)
    : m_bufferSize(bufferSize)
    , m_isInitialized(false)
    , m_accelPacketCount(0)
{
}

HighRateInterpolator::~HighRateInterpolator()
{
}

bool HighRateInterpolator::processPacket(const XsDataPacket& packet, XsDataPacket& interpolatedPacket)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // Check if packet contains sample time
    if (!packet.containsSampleTimeFine())
    {
        ROS_WARN_THROTTLE(5, "Packet does not contain SampleTimeFine");
        return false;
    }

    uint32_t sampleTimeFine = packet.sampleTimeFine();

    // Buffer incoming data based on packet type
    if (packet.containsOrientation())
    {
        XsQuaternion quat = packet.orientationQuaternion();
        addOrientationData(sampleTimeFine, quat);
    }

    if (packet.containsRateOfTurnHR())
    {
        XsVector gyro = packet.rateOfTurnHR();
        addRateOfTurnData(sampleTimeFine, gyro);
    }

    if (packet.containsAccelerationHR())
    {
        XsVector accel = packet.accelerationHR();
        addAccelerationData(sampleTimeFine, accel);

        // Increment acceleration packet counter
        m_accelPacketCount++;

        // AccelerationHR is our reference timeline
        // Try to interpolate orientation and rate of turn at this timestamp

        // Check if we have sufficient data for interpolation
        bool hasEnoughOrientation = m_orientationBuffer.size() >= 2;
        bool hasEnoughRateOfTurn = m_rateOfTurnBuffer.size() >= 2;

        if (!hasEnoughOrientation || !hasEnoughRateOfTurn)
        {
            // Only warn after initialization period
            if (m_accelPacketCount > INITIALIZATION_PACKET_THRESHOLD)
            {
                if (!hasEnoughOrientation)
                {
                    ROS_WARN_THROTTLE(5,
                        "Insufficient orientation data for interpolation (need at least 2, have %zu). "
                        "Check if orientation output is enabled on the MTi device.",
                        m_orientationBuffer.size());
                }

                if (!hasEnoughRateOfTurn)
                {
                    ROS_WARN_THROTTLE(5,
                        "Insufficient rate of turn data for interpolation (need at least 2, have %zu). "
                        "Check if RateOfTurnHR output is enabled on the MTi device.",
                        m_rateOfTurnBuffer.size());
                }
            }
            else
            {
                if (!m_isInitialized && hasEnoughOrientation && hasEnoughRateOfTurn)
                {
                    m_isInitialized = true;
                    ROS_INFO(
                        "High-rate interpolation initialized successfully. "
                        "Orientation buffer: %zu, RateOfTurn buffer: %zu, Acceleration buffer: %zu",
                        m_orientationBuffer.size(), m_rateOfTurnBuffer.size(), m_accelerationBuffer.size());
                }
            }

            return false;
        }

        // Mark as initialized if we haven't already
        if (!m_isInitialized)
        {
            m_isInitialized = true;
            ROS_INFO(
                "High-rate interpolation initialized successfully. "
                "Orientation buffer: %zu, RateOfTurn buffer: %zu, Acceleration buffer: %zu",
                m_orientationBuffer.size(), m_rateOfTurnBuffer.size(), m_accelerationBuffer.size());
        }

        // Create interpolated packet
        interpolatedPacket = packet; // Start with the acceleration packet as base

        // Interpolate orientation
        XsQuaternion interpolatedQuat;
        if (interpolateOrientation(sampleTimeFine, interpolatedQuat))
        {
            interpolatedPacket.setOrientationQuaternion(interpolatedQuat, XDI_CoordSysEnu);
            interpolatedPacket.setCalibratedAcceleration(accel);
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Failed to interpolate orientation at time %u", sampleTimeFine);
        }

        // Interpolate rate of turn
        XsVector interpolatedGyro;
        if (interpolateRateOfTurn(sampleTimeFine, interpolatedGyro))
        {
            interpolatedPacket.setRateOfTurnHR(interpolatedGyro);
            interpolatedPacket.setCalibratedGyroscopeData(interpolatedGyro);
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Failed to interpolate rate of turn at time %u", sampleTimeFine);
        }

        // Clean old data to prevent buffer overflow
        cleanOldData(sampleTimeFine);

        return true;
    }

    return false;
}

void HighRateInterpolator::addOrientationData(uint32_t sampleTimeFine, const XsQuaternion& quat)
{
    TimedOrientation data;
    data.sampleTimeFine = sampleTimeFine;
    data.quaternion = quat;

    m_orientationBuffer.push_back(data);

    while (m_orientationBuffer.size() > m_bufferSize)
    {
        m_orientationBuffer.pop_front();
    }
}

void HighRateInterpolator::addRateOfTurnData(uint32_t sampleTimeFine, const XsVector& gyro)
{
    TimedRateOfTurn data;
    data.sampleTimeFine = sampleTimeFine;
    data.gyro = gyro;

    m_rateOfTurnBuffer.push_back(data);

    while (m_rateOfTurnBuffer.size() > m_bufferSize)
    {
        m_rateOfTurnBuffer.pop_front();
    }
}

void HighRateInterpolator::addAccelerationData(uint32_t sampleTimeFine, const XsVector& accel)
{
    TimedAcceleration data;
    data.sampleTimeFine = sampleTimeFine;
    data.accel = accel;

    m_accelerationBuffer.push_back(data);

    while (m_accelerationBuffer.size() > m_bufferSize)
    {
        m_accelerationBuffer.pop_front();
    }
}

bool HighRateInterpolator::interpolateOrientation(uint32_t targetTime, XsQuaternion& result)
{
    if (m_orientationBuffer.size() < 2)
    {
        return false;
    }

    const TimedOrientation* before = nullptr;
    const TimedOrientation* after = nullptr;

    for (size_t i = 0; i < m_orientationBuffer.size() - 1; ++i)
    {
        int64_t diff1 = computeTimeDifference(m_orientationBuffer[i].sampleTimeFine, targetTime);
        int64_t diff2 = computeTimeDifference(m_orientationBuffer[i + 1].sampleTimeFine, targetTime);

        if (diff1 <= 0 && diff2 >= 0)
        {
            before = &m_orientationBuffer[i];
            after = &m_orientationBuffer[i + 1];
            break;
        }
    }

    if (!before || !after)
    {
        if (m_orientationBuffer.size() >= 2)
        {
            before = &m_orientationBuffer[m_orientationBuffer.size() - 2];
            after = &m_orientationBuffer[m_orientationBuffer.size() - 1];
        }
        else
        {
            return false;
        }
    }

    double t = computeInterpolationFactor(targetTime, before->sampleTimeFine, after->sampleTimeFine);
    t = std::max(0.0, std::min(2.0, t));
    result = slerp(before->quaternion, after->quaternion, t);

    return true;
}

bool HighRateInterpolator::interpolateRateOfTurn(uint32_t targetTime, XsVector& result)
{
    if (m_rateOfTurnBuffer.size() < 2)
    {
        return false;
    }

    const TimedRateOfTurn* before = nullptr;
    const TimedRateOfTurn* after = nullptr;

    for (size_t i = 0; i < m_rateOfTurnBuffer.size() - 1; ++i)
    {
        int64_t diff1 = computeTimeDifference(m_rateOfTurnBuffer[i].sampleTimeFine, targetTime);
        int64_t diff2 = computeTimeDifference(m_rateOfTurnBuffer[i + 1].sampleTimeFine, targetTime);

        if (diff1 <= 0 && diff2 >= 0)
        {
            before = &m_rateOfTurnBuffer[i];
            after = &m_rateOfTurnBuffer[i + 1];
            break;
        }
    }

    if (!before || !after)
    {
        if (m_rateOfTurnBuffer.size() >= 2)
        {
            before = &m_rateOfTurnBuffer[m_rateOfTurnBuffer.size() - 2];
            after = &m_rateOfTurnBuffer[m_rateOfTurnBuffer.size() - 1];
        }
        else
        {
            return false;
        }
    }

    double t = computeInterpolationFactor(targetTime, before->sampleTimeFine, after->sampleTimeFine);
    t = std::max(0.0, std::min(2.0, t));
    result = lerp(before->gyro, after->gyro, t);

    return true;
}

int64_t HighRateInterpolator::computeTimeDifference(uint32_t t1, uint32_t t2)
{
    int64_t diff = static_cast<int64_t>(t2) - static_cast<int64_t>(t1);

    if (diff > WRAP_THRESHOLD)
    {
        diff -= (static_cast<int64_t>(UINT32_MAX_VALUE) + 1);
    }
    else if (diff < -WRAP_THRESHOLD)
    {
        diff += (static_cast<int64_t>(UINT32_MAX_VALUE) + 1);
    }

    return diff;
}

double HighRateInterpolator::computeInterpolationFactor(uint32_t targetTime, uint32_t t1, uint32_t t2)
{
    int64_t totalDiff = computeTimeDifference(t1, t2);

    if (totalDiff == 0)
    {
        return 0.0;
    }

    int64_t targetDiff = computeTimeDifference(t1, targetTime);

    return static_cast<double>(targetDiff) / static_cast<double>(totalDiff);
}

XsQuaternion HighRateInterpolator::slerp(const XsQuaternion& q1, const XsQuaternion& q2, double t)
{
    Eigen::Quaterniond eigen_q1(q1.w(), q1.x(), q1.y(), q1.z());
    Eigen::Quaterniond eigen_q2(q2.w(), q2.x(), q2.y(), q2.z());

    Eigen::Quaterniond eigen_result = eigen_q1.slerp(t, eigen_q2);

    XsQuaternion result(eigen_result.w(), eigen_result.x(), eigen_result.y(), eigen_result.z());

    return result;
}

XsVector HighRateInterpolator::lerp(const XsVector& v1, const XsVector& v2, double t)
{
    XsVector result(3);

    for (int i = 0; i < 3; ++i)
    {
        result[i] = v1[i] + t * (v2[i] - v1[i]);
    }

    return result;
}

void HighRateInterpolator::cleanOldData(uint32_t currentTime)
{
    const int64_t TIME_WINDOW = 1000000; // 1 second in microseconds

    while (!m_orientationBuffer.empty())
    {
        int64_t diff = computeTimeDifference(m_orientationBuffer.front().sampleTimeFine, currentTime);
        if (diff < -TIME_WINDOW)
        {
            m_orientationBuffer.pop_front();
        }
        else
        {
            break;
        }
    }

    while (!m_rateOfTurnBuffer.empty())
    {
        int64_t diff = computeTimeDifference(m_rateOfTurnBuffer.front().sampleTimeFine, currentTime);
        if (diff < -TIME_WINDOW)
        {
            m_rateOfTurnBuffer.pop_front();
        }
        else
        {
            break;
        }
    }

    while (!m_accelerationBuffer.empty())
    {
        int64_t diff = computeTimeDifference(m_accelerationBuffer.front().sampleTimeFine, currentTime);
        if (diff < -TIME_WINDOW)
        {
            m_accelerationBuffer.pop_front();
        }
        else
        {
            break;
        }
    }
}
