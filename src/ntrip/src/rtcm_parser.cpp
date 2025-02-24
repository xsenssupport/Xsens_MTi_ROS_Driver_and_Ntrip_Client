#include "ntrip_client/rtcm_parser.h"
#include <sstream>
#include <iomanip>

namespace ntrip_client
{

    RtcmParser::RtcmParser(ros::Publisher &rtcm_pub, bool debug)
        : rtcm_pub_(rtcm_pub),
          bytes_required_(kRtcm3HeaderLength),
          in_message_(false),
          debug_(debug)
    {
    }

    void RtcmParser::ProcessData(const uint8_t *data, size_t length)
    {
        if (debug_)
        {
            LogDebug("Received " + std::to_string(length) + " bytes");
            LogDebug("Raw data: " + ToHexString(data, length));
        }

        raw_buffer_.insert(raw_buffer_.end(), data, data + length);
        ProcessBuffer();
    }

    size_t RtcmParser::PublishPendingMessages() {
        size_t count = 0;
        while (!valid_messages_.empty()) {
            if (debug_) {
                LogDebug("Publishing RTCM message with " +
                         std::to_string(valid_messages_.front().data.size()) + " bytes");
            }
            rtcm_pub_.publish(valid_messages_.front());
            valid_messages_.pop_front();
            count++;
        }
        return count;
    }

    void RtcmParser::ProcessBuffer()
    {
        while (true)
        {
            if (raw_buffer_.size() < bytes_required_)
            {
                if (debug_)
                {
                    LogDebug("Buffer size " + std::to_string(raw_buffer_.size()) +
                             " < required " + std::to_string(bytes_required_) + " bytes");
                }
                return;
            }

            if (!in_message_)
            {
                // Search for message start
                size_t start_pos = FindNextMessage(raw_buffer_);
                if (start_pos == std::string::npos)
                {
                    if (!raw_buffer_.empty())
                    {
                        if (debug_)
                        {
                            LogDebug("No valid message start found, keeping last " +
                                     std::to_string(kRtcm3HeaderLength - 1) + " bytes");
                        }
                        raw_buffer_.erase(raw_buffer_.begin(),
                                          raw_buffer_.end() - kRtcm3HeaderLength + 1);
                    }
                    return;
                }

                // Remove invalid data
                if (start_pos > 0)
                {
                    LogWarn("Discarded " + std::to_string(start_pos) + " bytes of invalid data");
                    raw_buffer_.erase(raw_buffer_.begin(),
                                      raw_buffer_.begin() + start_pos);
                }

                in_message_ = true;
            }

            // Parse message length
            const uint16_t payload_length = ((raw_buffer_[1] & 0x03) << 8) | raw_buffer_[2];
            const size_t total_length = kRtcm3HeaderLength + payload_length + kCrc24qLength;

            if (raw_buffer_.size() < total_length)
            {
                if (debug_)
                {
                    LogDebug("Incomplete message: have " + std::to_string(raw_buffer_.size()) +
                             " bytes, need " + std::to_string(total_length));
                }
                bytes_required_ = total_length;
                return;
            }

            // Extract complete message
            const uint8_t *msg_start = raw_buffer_.data();
            if (ValidateCRC24Q(msg_start, total_length))
            {
                // Get message type for debugging
                uint16_t msg_type = ((msg_start[1] & 0xFC) << 2) | ((msg_start[2] & 0xF0) >> 4);

                if (debug_)
                {
                    LogDebug("Valid message: type=" + std::to_string(msg_type) +
                             " (" + GetMessageDescription(msg_type) +
                             "), length=" + std::to_string(payload_length));
                    LogDebug("Message data: " +
                             ToHexString(msg_start, std::min(total_length, size_t(32))));
                }

                mavros_msgs::RTCM rtcm_msg;
                rtcm_msg.header.stamp = ros::Time::now();
                rtcm_msg.data.assign(msg_start, msg_start + total_length);
                valid_messages_.push_back(rtcm_msg);
            }
            else
            {
                LogWarn("Invalid CRC detected, dropping message of length " +
                        std::to_string(total_length));
            }

            // Remove processed data
            raw_buffer_.erase(raw_buffer_.begin(),
                              raw_buffer_.begin() + total_length);
            in_message_ = false;
            bytes_required_ = kRtcm3HeaderLength;
        }
    }

    size_t RtcmParser::FindNextMessage(const std::vector<uint8_t> &buffer) const
    {
        for (size_t i = 0; i < buffer.size() - kRtcm3HeaderLength + 1; ++i)
        {
            if (buffer[i] == kRtcm3Preamble)
            {
                // Validate length field
                const uint16_t payload_length = ((buffer[i + 1] & 0x03) << 8) | buffer[i + 2];
                if (payload_length <= kMaxPayloadLength)
                {
                    if (debug_)
                    {
                        LogDebug("Found potential message at offset " + std::to_string(i) +
                                 " with payload length " + std::to_string(payload_length));
                    }
                    return i;
                }
                else
                {
                    if (debug_)
                    {
                        LogDebug("Invalid payload length " + std::to_string(payload_length) +
                                 " at offset " + std::to_string(i));
                    }
                }
            }
        }
        return std::string::npos;
    }

    bool RtcmParser::ValidateCRC24Q(const uint8_t *data, size_t length) const
    {
        if (length < kRtcm3HeaderLength + kCrc24qLength)
        {
            return false;
        }

        crc_24q_type crc_24q;
        crc_24q.process_bytes(data, length - kCrc24qLength);

        const uint32_t computed_crc = crc_24q.checksum();
        const uint32_t received_crc = (data[length - 3] << 16) |
                                      (data[length - 2] << 8) |
                                      data[length - 1];

        if (debug_)
        {
            std::stringstream ss;
            ss << std::hex << std::uppercase
               << "CRC check - Computed: 0x" << std::setfill('0') << std::setw(6)
               << computed_crc << ", Received: 0x" << std::setfill('0') << std::setw(6)
               << received_crc;
            LogDebug(ss.str());
        }

        return computed_crc == received_crc;
    }

    std::string RtcmParser::ToHexString(const uint8_t *data, size_t length)
    {
        std::stringstream ss;
        for (size_t i = 0; i < length; ++i)
        {
            if (i > 0)
                ss << " ";
            ss << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
               << static_cast<int>(data[i]);
        }
        if (length < 32)
        {
            ss << "...";
        }
        return ss.str();
    }

    std::string RtcmParser::GetMessageDescription(uint16_t msg_type) const
    {
        if (msg_type >= 1071 && msg_type <= 1077)
            return "GPS MSM" + std::to_string(msg_type - 1070);
        else if (msg_type >= 1081 && msg_type <= 1087)
            return "GLONASS MSM" + std::to_string(msg_type - 1080);
        else if (msg_type >= 1091 && msg_type <= 1097)
            return "Galileo MSM" + std::to_string(msg_type - 1090);
        else if (msg_type >= 1121 && msg_type <= 1127)
            return "BeiDou MSM" + std::to_string(msg_type - 1120);
        else if (msg_type >= 1001 && msg_type <= 1004)
            return "GPS RTK";
        else if (msg_type >= 1009 && msg_type <= 1012)
            return "GLONASS RTK";
        else if (msg_type == 1019)
            return "GPS Ephemeris";
        else if (msg_type == 1020)
            return "GLONASS Ephemeris";
        else if (msg_type == 1042)
            return "BeiDou Ephemeris";
        else if (msg_type == 1045)
            return "Galileo Ephemeris";
        return "Other RTCM3 message";
    }

    void RtcmParser::LogDebug(const std::string &msg) const
    {
        if (debug_)
        {
            ROS_DEBUG_STREAM("RTCMParser: " << msg);
        }
    }

    void RtcmParser::LogWarn(const std::string &msg) const
    {
        ROS_WARN_STREAM("RTCMParser: " << msg);
    }

    void RtcmParser::LogError(const std::string &msg) const
    {
        ROS_ERROR_STREAM("RTCMParser: " << msg);
    }

} // namespace ntrip_client