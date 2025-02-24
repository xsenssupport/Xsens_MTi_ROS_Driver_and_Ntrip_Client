#ifndef NTRIP_CLIENT_RTCM_PARSER_H_
#define NTRIP_CLIENT_RTCM_PARSER_H_

#include <vector>
#include <deque>
#include <ros/ros.h>
#include <mavros_msgs/RTCM.h>
#include <boost/crc.hpp>

namespace ntrip_client
{

    class RtcmParser
    {
    public:
        explicit RtcmParser(ros::Publisher &rtcm_pub, bool debug = false);

        // Process new data and optionally publish messages
        void ProcessData(const uint8_t *data, size_t length);

        // Publish any pending messages
        size_t PublishPendingMessages();

    private:
        static constexpr uint8_t kRtcm3Preamble = 0xD3;
        static constexpr size_t kRtcm3HeaderLength = 3;
        static constexpr size_t kCrc24qLength = 3;
        static constexpr size_t kMaxPayloadLength = 1023; // RTCM 3.x maximum length

        void ProcessBuffer();
        bool ValidateCRC24Q(const uint8_t *data, size_t length) const;
        size_t FindNextMessage(const std::vector<uint8_t> &buffer) const;
        std::string GetMessageDescription(uint16_t msg_type) const;
        std::string ToHexString(const uint8_t *data, size_t length);

        // Logging helpers
        void LogDebug(const std::string &msg) const;
        void LogWarn(const std::string &msg) const;
        void LogError(const std::string &msg) const;

        ros::Publisher &rtcm_pub_;
        std::vector<uint8_t> raw_buffer_;
        std::deque<mavros_msgs::RTCM> valid_messages_;

        size_t bytes_required_;
        bool in_message_;
        bool debug_;

        // CRC calculator
        typedef boost::crc_optimal<24, 0x1864CFB, 0x0, 0x0, false, false> crc_24q_type;
    };

} // namespace ntrip_client

#endif // NTRIP_CLIENT_RTCM_PARSER_H_