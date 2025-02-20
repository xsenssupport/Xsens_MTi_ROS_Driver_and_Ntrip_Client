#ifndef NTRIP_CLIENT_NTRIP_CLIENT_H_
#define NTRIP_CLIENT_NTRIP_CLIENT_H_

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <mavros_msgs/RTCM.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <queue>
#include <mutex>
#include <memory>
#include <iomanip>
#include "rtcm_parser.h"

namespace ntrip_client
{
    // declare constant before the class.
    const int NTRIP_CONNECT_TIMEOUT = 300;

    class NtripClient
    {
    public:
        NtripClient(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        ~NtripClient();
        bool Start();
        void Stop();

    private:
        // ROS related
        ros::NodeHandle &nh_;
        ros::NodeHandle &private_nh_;
        ros::Subscriber nmea_sub_;
        ros::Publisher rtcm_pub_;
        ros::Timer connection_timer_;
        ros::Timer status_timer_;

        // Parameters
        std::string host_;
        std::string port_;
        std::string mountpoint_;
        std::string username_;
        std::string password_;
        double nmea_input_rate_;
        double update_rate_;
        double reconnect_delay_;
        int max_reconnect_attempts_;

        // Networking
        boost::asio::io_service io_service_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::deadline_timer connect_timeout_timer_;
        boost::asio::streambuf response_;
        std::unique_ptr<boost::thread> io_thread_;
        std::vector<char> receive_buffer_;
        bool is_connected_;
        bool should_exit_;
        std::mutex socket_mutex_;

        // Statistics
        size_t bytes_received_;
        size_t rtcm_messages_count_;
        ros::Time last_nmea_time_;
        ros::Time last_rtcm_time_;
        int reconnect_attempts_;

        // Message queues
        std::mutex nmea_mutex_;

        // Default GGA message
        std::string default_gga_;
        int nmea_msg_counter_;
        int nmea_skip_count_; // How many messages to skip (calculated from input rate and desired rate)
        bool send_default_gga_;
        bool output_rtcm_details_;

        int port_number_;

        std::unique_ptr<boost::asio::io_service::work> work_;

        ros::Publisher diagnostic_pub_;
        bool debug_;

        std::unique_ptr<RtcmParser> rtcm_parser_;

        // Methods
        bool Initialize();
        void Connect();
        void Disconnect();
        void HandleNmeaMessage(const nmea_msgs::Sentence::ConstPtr &msg);
        void CheckConnection(const ros::TimerEvent &event);
        void PublishStatus(const ros::TimerEvent &event);
        bool SendHttpRequest();
        void ReadData();
        void HandleRead(const boost::system::error_code &error, size_t bytes_transferred);
        std::string CreateAuthHeader() const;
        void HandleError(const std::string &error_msg, bool fatal = false);
        bool ValidateParameters() const;
    };

} // namespace ntrip_client

#endif // NTRIP_CLIENT_NTRIP_CLIENT_H_