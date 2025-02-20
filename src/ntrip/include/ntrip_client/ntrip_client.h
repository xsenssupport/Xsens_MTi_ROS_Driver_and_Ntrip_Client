#ifndef NTRIP_CLIENT_NTRIP_CLIENT_HPP_
#define NTRIP_CLIENT_NTRIP_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/thread.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ntrip_client/rtcm_parser.h"

namespace ntrip_client
{

    // Declare constant before the class.
    const int kNtripConnectTimeout = 300;

    class NtripClient : public rclcpp::Node
    {
    public:
        explicit NtripClient();
        ~NtripClient();

        bool Start();
        void Stop();

    private:
        // ROS related
        rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;
        rclcpp::Publisher<mavros_msgs::msg::RTCM>::SharedPtr rtcm_pub_;
        rclcpp::TimerBase::SharedPtr connection_timer_;
        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;

        // Statistics
        size_t bytes_received_;
        size_t rtcm_messages_count_;
        rclcpp::Time last_nmea_time_;
        rclcpp::Time last_rtcm_time_;
        int reconnect_attempts_;
        int nmea_msg_counter_;

        // Control flags
        bool debug_;
        bool is_connected_;
        bool should_exit_;


        // Networking
        boost::asio::io_service io_service_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::deadline_timer connect_timeout_timer_;
        boost::asio::streambuf response_;
        std::unique_ptr<boost::thread> io_thread_;
        std::vector<char> receive_buffer_;
        std::unique_ptr<boost::asio::io_service::work> work_;

        // Parameters
        std::string host_;
        std::string port_;
        std::string mountpoint_;
        std::string username_;
        std::string password_;
        std::string default_gga_;
        double nmea_input_rate_;
        double update_rate_;
        double reconnect_delay_;
        int max_reconnect_attempts_;
        int port_number_;
        int nmea_skip_count_;

        // Mutexes
        std::mutex socket_mutex_;
        std::mutex nmea_mutex_;

        // RTCM Parser
        std::unique_ptr<RtcmParser> rtcm_parser_;

        // Methods
        bool Initialize();
        void Connect();
        void Disconnect();
        void HandleNmeaMessage(const nmea_msgs::msg::Sentence::SharedPtr msg);
        void CheckConnection();
        void PublishStatus();
        bool SendHttpRequest();
        void ReadData();
        void HandleRead(const boost::system::error_code &error,
                        size_t bytes_transferred);
        std::string CreateAuthHeader() const;
        void HandleError(const std::string &error_msg, bool fatal = false);
        bool ValidateParameters() const;
        void DeclareParameters();
    };

} // namespace ntrip_client

#endif // NTRIP_CLIENT_NTRIP_CLIENT_HPP_