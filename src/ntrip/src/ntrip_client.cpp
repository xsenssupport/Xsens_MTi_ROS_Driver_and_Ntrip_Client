#include "ntrip_client/ntrip_client.h"
#include <boost/algorithm/string.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>

namespace ntrip_client
{

    NtripClient::NtripClient(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      socket_(io_service_),
      connect_timeout_timer_(io_service_),
      is_connected_(false),
      should_exit_(false),
      bytes_received_(0),
      rtcm_messages_count_(0),
      reconnect_attempts_(0),
      debug_(false),
      nmea_msg_counter_(0)
    {
        receive_buffer_.resize(4096);

        // make sure the socket is closed.
        if (socket_.is_open())
        {
            socket_.close();
        }

    }

    NtripClient::~NtripClient()
    {
        Stop();
    }

    bool NtripClient::Start()
    {
        if (!Initialize())
        {
            return false;
        }

        // a work instance to keep io_service running.
        work_ = std::make_unique<boost::asio::io_service::work>(io_service_);

        // Start IO service in separate thread
        io_thread_ = std::make_unique<boost::thread>(
            boost::bind(&boost::asio::io_service::run, &io_service_));

        ROS_INFO("IO service thread started");

        // Initial connection attempt
        Connect();
        return true;
    }

    void NtripClient::Stop()
    {
        should_exit_ = true;
        Disconnect();

        if (io_thread_ && io_thread_->joinable())
        {
            io_service_.stop();
            io_thread_->join();
        }
    }

    bool NtripClient::Initialize()
    {
        try
        {
            // get necessary parameters
            if (!private_nh_.getParam("host", host_))
            {
                ROS_ERROR("Parameter 'host' not set!");
                return false;
            }

            // get the port number
            int port_number;
            if (!private_nh_.getParam("port", port_number))
            {
                ROS_ERROR("Parameter 'port' not set!");
                return false;
            }
            port_ = std::to_string(port_number);
            port_number_ = port_number;

            if (!private_nh_.getParam("mountpoint", mountpoint_))
            {
                ROS_ERROR("Parameter 'mountpoint' not set!");
                return false;
            }

            if (!private_nh_.getParam("username", username_))
            {
                ROS_ERROR("Parameter 'username' not set!");
                return false;
            }

            if (!private_nh_.getParam("password", password_))
            {
                ROS_ERROR("Parameter 'password' not set!");
                return false;
            }

            // get optional parameters, otherwise use default values.
            private_nh_.param<double>("nmea_input_rate", nmea_input_rate_, 4.0);
            private_nh_.param<double>("update_rate", update_rate_, 1.0);
            private_nh_.param<double>("reconnect_delay", reconnect_delay_, 5.0);
            private_nh_.param<int>("max_reconnect_attempts", max_reconnect_attempts_, 0);
            private_nh_.param<bool>("debug", debug_, false);
            private_nh_.param<bool>("send_default_gga", send_default_gga_, true);
            private_nh_.param<bool>("output_rtcm_details", output_rtcm_details_, false);

            // print the parameters to confirm they are correct.
            ROS_INFO("Initializing NTRIP client with following parameters:");
            ROS_INFO("Host: %s", host_.c_str());
            ROS_INFO("Port: %d", port_number_);
            ROS_INFO("Mountpoint: %s", mountpoint_.c_str());
            ROS_INFO("Username: %s", username_.empty() ? "not set" : "****");
            ROS_INFO("Password: %s", password_.empty() ? "not set" : "****");
            ROS_INFO("NMEA input rate: %.1f Hz", nmea_input_rate_); 
            ROS_INFO("Update rate: %.1f Hz", update_rate_);
            ROS_INFO("Reconnect delay: %.1f seconds", reconnect_delay_);
            ROS_INFO("Send default GGA: %s", send_default_gga_ ? "true" : "false");

            if (debug_)
            {
                ROS_INFO("Debug mode enabled");
            }

            // validate the parameters
            if (!ValidateParameters())
            {
                return false;
            }



            // Initialize message counter
            nmea_msg_counter_ = 0;

            default_gga_ = "$GPGGA,000000.000,0000.0000,N,00000.0000,E,1,12,1.0,0.0,M,0.0,M,,*68\r\n";

            // Calculate skip count using nmea_input_rate_
            nmea_skip_count_ = static_cast<int>(std::round(nmea_input_rate_ / update_rate_)) - 1;

            ROS_INFO("NMEA downsampling: input rate %.1f Hz, desired rate %.1f Hz, will skip %d messages",
                    nmea_input_rate_, update_rate_, nmea_skip_count_);

            // set the publisher and subscriber.
            rtcm_pub_ = nh_.advertise<mavros_msgs::RTCM>("rtcm", 10);
            diagnostic_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/ntrip/diagnostics", 10);
            nmea_sub_ = nh_.subscribe("nmea", 10, &NtripClient::HandleNmeaMessage, this);

            // Create RtcmParser AFTER parameters are loaded
            rtcm_parser_ = std::make_unique<RtcmParser>(rtcm_pub_, debug_);

            // set the timer.
            connection_timer_ = nh_.createTimer(
                ros::Duration(reconnect_delay_),
                &NtripClient::CheckConnection, this);

            status_timer_ = nh_.createTimer(
                ros::Duration(1.0),
                &NtripClient::PublishStatus, this);

            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error initializing NTRIP client: %s", e.what());
            return false;
        }
    }

    void NtripClient::Connect()
    {
        if (is_connected_ || should_exit_)
            return;

        ROS_INFO("Attempting to connect to NTRIP caster...");

        try
        {
            // make sure the socket is closed.
            if (socket_.is_open())
            {
                socket_.close();
            }

            // re-open socket
            socket_ = boost::asio::ip::tcp::socket(io_service_);

            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::asio::ip::tcp::resolver::query query(host_, port_);

            ROS_INFO("Resolving hostname...");
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

            ROS_INFO("Connecting to endpoint...");

            // set connection timeout.
            connect_timeout_timer_.expires_from_now(boost::posix_time::seconds(NTRIP_CONNECT_TIMEOUT));
            connect_timeout_timer_.async_wait(
                [this](const boost::system::error_code &error)
                {
                    if (!error)
                    {
                        ROS_ERROR("Connection timeout");
                        boost::system::error_code ec;
                        socket_.close(ec);
                    }
                });

            // sync connection.
            boost::system::error_code connect_error;
            boost::asio::connect(socket_, endpoint_iterator, connect_error);

            if (connect_error)
            {
                connect_timeout_timer_.cancel();
                HandleError("Connection failed: " + connect_error.message());
                return;
            }

            // set socket option after successful connection.
            boost::asio::socket_base::keep_alive option(true);
            socket_.set_option(option);

            ROS_INFO("Connected to server, sending HTTP request...");
            if (SendHttpRequest())
            {
                connect_timeout_timer_.cancel();
                is_connected_ = true;
                reconnect_attempts_ = 0;
                ROS_INFO("Successfully connected to NTRIP caster at %s:%s", host_.c_str(), port_.c_str());
            }
        }
        catch (const boost::system::system_error &e)
        {
            HandleError("Connection error: " + std::string(e.what()));
        }
    }

    void NtripClient::Disconnect()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (socket_.is_open())
        {
            boost::system::error_code error;
            socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
            if (error && error != boost::asio::error::not_connected)
            {
                ROS_WARN("Socket shutdown error: %s", error.message().c_str());
            }
            socket_.close(error);
            if (error)
            {
                ROS_WARN("Socket close error: %s", error.message().c_str());
            }
        }
        is_connected_ = false;
    }

    bool NtripClient::SendHttpRequest()
    {
        std::stringstream request;
        request << "GET /" << mountpoint_ << " HTTP/1.1\r\n"
                << "Host: " << host_ << ":" << port_ << "\r\n"
                << "Ntrip-Version: Ntrip/2.0\r\n"
                << "User-Agent: NTRIP ROS Client/1.0\r\n";

        if (!username_.empty() || !password_.empty())
        {
            request << "Authorization: Basic " << CreateAuthHeader() << "\r\n";
        }

        request << "Connection: close\r\n\r\n";

        ROS_INFO("Sending HTTP request to NTRIP server");

        boost::system::error_code error;
        boost::asio::write(socket_, boost::asio::buffer(request.str()), error);

        if (error)
        {
            HandleError("Failed to send HTTP request: " + error.message());
            return false;
        }

        // read the response
        boost::asio::streambuf response;
        boost::system::error_code read_error;
        size_t bytes = boost::asio::read_until(socket_, response, "\r\n", read_error);

        if (read_error)
        {
            HandleError("Failed to read response: " + read_error.message());
            return false;
        }

        std::string response_line{
            boost::asio::buffers_begin(response.data()),
            boost::asio::buffers_begin(response.data()) + bytes};
        response.consume(bytes);

        ROS_INFO("Received response: %s", response_line.c_str());

        if (response_line.find("200") != std::string::npos)
        {
            ROS_INFO("NTRIP connection established successfully");
            is_connected_ = true;
            reconnect_attempts_ = 0;

            ReadData();
            return true;
        }
        else
        {
            HandleError("Unexpected server response: " + response_line);
            return false;
        }
    }

    void NtripClient::ReadData()
    {
        socket_.async_read_some(
            boost::asio::buffer(receive_buffer_),
            boost::bind(&NtripClient::HandleRead, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void NtripClient::HandleRead(const boost::system::error_code &error, size_t bytes_transferred)
    {
        if (!error)
        {
            if (debug_)
            {
                ROS_DEBUG("Received %zu bytes from NTRIP server", bytes_transferred);
            }

            // Update bytes received
            bytes_received_ += bytes_transferred;

            rtcm_parser_->ProcessData(
                reinterpret_cast<const uint8_t *>(receive_buffer_.data()),
                bytes_transferred);

            // Update RTCM message count
            size_t published = rtcm_parser_->PublishPendingMessages();
            rtcm_messages_count_ += published;

            // continue to read data
            ReadData();
        }
        else if (error == boost::asio::error::eof)
        {
            ROS_INFO("Connection closed by server");
            Disconnect();
        }
        else
        {
            HandleError("Read error: " + error.message());
            Disconnect();
        }
    }

    void NtripClient::HandleNmeaMessage(const nmea_msgs::Sentence::ConstPtr &msg)
    {
        // Only process GGA messages
        if (msg->sentence.substr(0, 6) != "$GPGGA" && msg->sentence.substr(0, 6) != "$GNGGA")
        {
            return;
        }

        if (debug_)
        {
            ROS_DEBUG("Received GGA message: %s", msg->sentence.c_str());
        }

        // Increment counter and check if we should process this message
        nmea_msg_counter_ = (nmea_msg_counter_ + 1) % (nmea_skip_count_ + 1);

        // Only process message when counter is 0
        if (nmea_msg_counter_ == 0 && is_connected_)
        {
            try
            {
                std::string nmea_msg = msg->sentence + "\r\n";

                ROS_INFO("Sending GGA message: %s", msg->sentence.c_str());

                boost::system::error_code error;
                boost::asio::write(socket_, boost::asio::buffer(nmea_msg), error);

                if (error)
                {
                    HandleError("Failed to send GGA message: " + error.message(), false);
                }
                else if (debug_)
                {
                    ROS_DEBUG("Successfully sent GGA message");
                }
            }
            catch (const std::exception &e)
            {
                HandleError("GGA send error: " + std::string(e.what()), false);
            }
        }
    }

    void NtripClient::CheckConnection(const ros::TimerEvent &event)
    {
        if (!is_connected_)
        {
            ROS_INFO_THROTTLE(10, "Attempting to reconnect...");
            Connect();
        }
    }

    std::string NtripClient::CreateAuthHeader() const
    {
    
        std::string auth_string = username_ + ":" + password_;
        std::string encoded;
        
        encoded.resize(boost::beast::detail::base64::encoded_size(auth_string.size()));
        int len = boost::beast::detail::base64::encode(&encoded[0], auth_string.data(), auth_string.size());
        
        encoded.resize(len);
        
        return encoded;
    }

    void NtripClient::HandleError(const std::string &error_msg, bool fatal)
    {
        ROS_ERROR_STREAM(error_msg);
        if (fatal)
        {
            is_connected_ = false;
            Disconnect();
        }
    }

    void NtripClient::PublishStatus(const ros::TimerEvent &event)
    {
        diagnostic_msgs::DiagnosticArray diag_array;
        diagnostic_msgs::DiagnosticStatus status;

        status.name = "NTRIP Client Status";
        status.hardware_id = host_ + ":" + port_;

        if (is_connected_)
        {
            status.level = diagnostic_msgs::DiagnosticStatus::OK;
            status.message = "Connected";
        }
        else
        {
            status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            status.message = "Disconnected";
        }

        diagnostic_msgs::KeyValue kv;

        kv.key = "Bytes Received";
        kv.value = std::to_string(bytes_received_);
        status.values.push_back(kv);

        kv.key = "RTCM Messages";
        kv.value = std::to_string(rtcm_messages_count_);
        status.values.push_back(kv);

        kv.key = "Reconnect Attempts";
        kv.value = std::to_string(reconnect_attempts_);
        status.values.push_back(kv);

        diag_array.header.stamp = ros::Time::now();
        diag_array.status.push_back(status);

        diagnostic_pub_.publish(diag_array);
    }

    bool NtripClient::ValidateParameters() const
    {
        if (host_.empty())
        {
            ROS_ERROR("Host must be specified!");
            return false;
        }

        if (port_number_ <= 0 || port_number_ > 65535)
        {
            ROS_ERROR("Invalid port number: %d", port_number_);
            return false;
        }

        if (mountpoint_.empty())
        {
            ROS_ERROR("Mountpoint must be specified!");
            return false;
        }

        if (username_.empty() || password_.empty())
        {
            ROS_ERROR("Username and password must be specified!");
            return false;
        }

        if (nmea_input_rate_ <= 0)
        {
            ROS_ERROR("NMEA input rate must be positive!");
            return false;
        }

        if (update_rate_ <= 0)
        {
            ROS_ERROR("Update rate must be positive!");
            return false;
        }

        if (update_rate_ > nmea_input_rate_)
        {
            ROS_ERROR("Update rate (%.1f Hz) cannot be higher than NMEA input rate (%.1f Hz)!",
                    update_rate_, nmea_input_rate_);
            return false;
        }

        if (reconnect_delay_ <= 0)
        {
            ROS_ERROR("Reconnect delay must be positive!");
            return false;
        }

        return true;
    }

} // namespace ntrip_client