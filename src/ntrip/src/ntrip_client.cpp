#include "ntrip_client/ntrip_client.h"

#include <boost/algorithm/string.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <iomanip>
#include <sstream>

namespace ntrip_client
{

  NtripClient::NtripClient()
      : Node("ntrip_client"),
        bytes_received_(0),
        rtcm_messages_count_(0),
        reconnect_attempts_(0),
        nmea_msg_counter_(0),
        debug_(false),
        is_connected_(false),
        should_exit_(false),
        socket_(io_service_),
        connect_timeout_timer_(io_service_)
  {
    receive_buffer_.resize(4096);

    // Make sure the socket is closed.
    if (socket_.is_open())
    {
      socket_.close();
    }

    DeclareParameters();
    Initialize();

  }

  NtripClient::~NtripClient() { Stop(); }

  void NtripClient::DeclareParameters()
  {
    // Declare all parameters
    this->declare_parameter("host", "");
    this->declare_parameter("port", 0);
    this->declare_parameter("mountpoint", "");
    this->declare_parameter("username", "");
    this->declare_parameter("password", "");
    this->declare_parameter("nmea_input_rate", 4.0);
    this->declare_parameter("update_rate", 1.0);
    this->declare_parameter("reconnect_delay", 5.0);
    this->declare_parameter("max_reconnect_attempts", 0);
    this->declare_parameter("debug", false);
  }

  bool NtripClient::Start()
  {
    // A work instance to keep io_service running.
    work_ = std::make_unique<boost::asio::io_service::work>(io_service_);

    // Start IO service in separate thread
    io_thread_ = std::make_unique<boost::thread>(
        boost::bind(&boost::asio::io_service::run, &io_service_));

    RCLCPP_INFO(this->get_logger(), "IO service thread started");

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
      // Get necessary parameters
      host_ = this->get_parameter("host").as_string();
      port_number_ = this->get_parameter("port").as_int();
      port_ = std::to_string(port_number_);
      mountpoint_ = this->get_parameter("mountpoint").as_string();
      username_ = this->get_parameter("username").as_string();
      password_ = this->get_parameter("password").as_string();

      // Get optional parameters
      nmea_input_rate_ = this->get_parameter("nmea_input_rate").as_double();
      update_rate_ = this->get_parameter("update_rate").as_double();
      reconnect_delay_ = this->get_parameter("reconnect_delay").as_double();
      max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
      debug_ = this->get_parameter("debug").as_bool();

      // Print the parameters
      RCLCPP_INFO(this->get_logger(), "Initializing NTRIP client with following parameters:");
      RCLCPP_INFO(this->get_logger(), "Host: %s", host_.c_str());
      RCLCPP_INFO(this->get_logger(), "Port: %d", port_number_);
      RCLCPP_INFO(this->get_logger(), "Mountpoint: %s", mountpoint_.c_str());
      RCLCPP_INFO(this->get_logger(), "Username: %s", username_.empty() ? "not set" : "****");
      RCLCPP_INFO(this->get_logger(), "Password: %s", password_.empty() ? "not set" : "****");
      RCLCPP_INFO(this->get_logger(), "NMEA input rate: %.1f Hz", nmea_input_rate_);
      RCLCPP_INFO(this->get_logger(), "Update rate: %.1f Hz", update_rate_);
      RCLCPP_INFO(this->get_logger(), "Reconnect delay: %.1f seconds", reconnect_delay_);

      if (debug_)
      {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
      }

      // Validate the parameters
      if (!ValidateParameters())
      {
        return false;
      }



      // Initialize message counter
      nmea_msg_counter_ = 0;
      default_gga_ = "$GPGGA,000000.000,0000.0000,N,00000.0000,E,1,12,1.0,0.0,M,0.0,M,,*68\r\n";

      // Calculate skip count using nmea_input_rate_
      nmea_skip_count_ = static_cast<int>(std::round(nmea_input_rate_ / update_rate_)) - 1;

      RCLCPP_INFO(this->get_logger(),
                  "NMEA downsampling: input rate %.1f Hz, desired rate %.1f Hz, will skip %d messages",
                  nmea_input_rate_, update_rate_, nmea_skip_count_);

      // Set up publishers and subscribers
      rtcm_pub_ = this->create_publisher<mavros_msgs::msg::RTCM>("rtcm", 10);
      diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/ntrip/diagnostics", 10);

      // Create RtcmParser AFTER parameters are loaded
      rtcm_parser_ = std::make_unique<RtcmParser>(rtcm_pub_, debug_);

      nmea_sub_ = this->create_subscription<nmea_msgs::msg::Sentence>(
          "nmea", 10,
          std::bind(&NtripClient::HandleNmeaMessage, this, std::placeholders::_1));

      // Set up timers
      connection_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(reconnect_delay_),
          std::bind(&NtripClient::CheckConnection, this));

      status_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0),
          std::bind(&NtripClient::PublishStatus, this));

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error initializing NTRIP client: %s", e.what());
      return false;
    }
  }

  void NtripClient::Connect()
  {
    if (is_connected_ || should_exit_)
      return;

    RCLCPP_INFO(this->get_logger(), "Attempting to connect to NTRIP caster...");

    try
    {
      // Make sure the socket is closed.
      if (socket_.is_open())
      {
        socket_.close();
      }

      // Re-open socket
      socket_ = boost::asio::ip::tcp::socket(io_service_);

      boost::asio::ip::tcp::resolver resolver(io_service_);
      boost::asio::ip::tcp::resolver::query query(host_, port_);

      RCLCPP_INFO(this->get_logger(), "Resolving hostname...");
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

      RCLCPP_INFO(this->get_logger(), "Connecting to endpoint...");

      // Set connection timeout.
      connect_timeout_timer_.expires_from_now(boost::posix_time::seconds(kNtripConnectTimeout));
      connect_timeout_timer_.async_wait(
          [this](const boost::system::error_code &error)
          {
            if (!error)
            {
              RCLCPP_ERROR(this->get_logger(), "Connection timeout");
              boost::system::error_code ec;
              socket_.close(ec);
            }
          });

      // Sync connection.
      boost::system::error_code connect_error;
      boost::asio::connect(socket_, endpoint_iterator, connect_error);

      if (connect_error)
      {
        connect_timeout_timer_.cancel();
        HandleError("Connection failed: " + connect_error.message());
        return;
      }

      // Set socket option after successful connection.
      boost::asio::socket_base::keep_alive option(true);
      socket_.set_option(option);

      RCLCPP_INFO(this->get_logger(), "Connected to server, sending HTTP request...");
      if (SendHttpRequest())
      {
        connect_timeout_timer_.cancel();
        is_connected_ = true;
        reconnect_attempts_ = 0;
        RCLCPP_INFO(this->get_logger(), "Successfully connected to NTRIP caster at %s:%s",
                    host_.c_str(), port_.c_str());
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
        RCLCPP_WARN(this->get_logger(), "Socket shutdown error: %s",
                    error.message().c_str());
      }
      socket_.close(error);
      if (error)
      {
        RCLCPP_WARN(this->get_logger(), "Socket close error: %s",
                    error.message().c_str());
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
            << "User-Agent: NTRIP ROS2 Client/1.0\r\n";

    if (!username_.empty() || !password_.empty())
    {
      request << "Authorization: Basic " << CreateAuthHeader() << "\r\n";
    }

    request << "Connection: close\r\n\r\n";

    RCLCPP_INFO(this->get_logger(), "Sending HTTP request to NTRIP server");

    boost::system::error_code error;
    boost::asio::write(socket_, boost::asio::buffer(request.str()), error);

    if (error)
    {
      HandleError("Failed to send HTTP request: " + error.message());
      return false;
    }

    // Read the response
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

    RCLCPP_INFO(this->get_logger(), "Received response: %s", response_line.c_str());

    if (response_line.find("200") != std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "NTRIP connection established successfully");
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
        boost::bind(&NtripClient::HandleRead, this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

  void NtripClient::HandleRead(const boost::system::error_code &error,
                               size_t bytes_transferred)
  {
    if (!error)
    {
      if (debug_)
      {
        RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes from NTRIP server",
                     bytes_transferred);
      }

      // Update bytes received
      bytes_received_ += bytes_transferred;

      rtcm_parser_->ProcessData(
          reinterpret_cast<const uint8_t *>(receive_buffer_.data()),
          bytes_transferred);

      // Update RTCM message count
      size_t published = rtcm_parser_->PublishPendingMessages();
      rtcm_messages_count_ += published;

      // Continue to read data
      ReadData();
    }
    else if (error == boost::asio::error::eof)
    {
      RCLCPP_INFO(this->get_logger(), "Connection closed by server");
      Disconnect();
    }
    else
    {
      HandleError("Read error: " + error.message());
      Disconnect();
    }
  }

  void NtripClient::HandleNmeaMessage(
      const nmea_msgs::msg::Sentence::SharedPtr msg)
  {
    // Only process GGA messages
    if (msg->sentence.substr(0, 6) != "$GPGGA" &&
        msg->sentence.substr(0, 6) != "$GNGGA")
    {
      return;
    }

    if (debug_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Received GGA message: %s",
                   msg->sentence.c_str());
    }

    // Increment counter and check if we should process this message
    nmea_msg_counter_ = (nmea_msg_counter_ + 1) % (nmea_skip_count_ + 1);

    // Only process message when counter is 0
    if (nmea_msg_counter_ == 0 && is_connected_)
    {
      try
      {
        std::string nmea_msg = msg->sentence + "\r\n";

        RCLCPP_INFO(this->get_logger(), "Sending GGA message: %s",
                    msg->sentence.c_str());

        boost::system::error_code error;
        boost::asio::write(socket_, boost::asio::buffer(nmea_msg), error);

        if (error)
        {
          HandleError("Failed to send GGA message: " + error.message(), false);
        }
        else if (debug_)
        {
          RCLCPP_DEBUG(this->get_logger(), "Successfully sent GGA message");
        }
      }
      catch (const std::exception &e)
      {
        HandleError("GGA send error: " + std::string(e.what()), false);
      }
    }
  }

  void NtripClient::CheckConnection()
  {
    if (!is_connected_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                           "Attempting to reconnect...");
      Connect();
    }
  }

  void NtripClient::PublishStatus()
  {
    auto diag_array = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_msgs::msg::DiagnosticStatus status;

    status.name = "NTRIP Client Status";
    status.hardware_id = host_ + ":" + port_;

    if (is_connected_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Connected";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Disconnected";
    }

    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "Bytes Received";
    kv.value = std::to_string(bytes_received_);
    status.values.push_back(kv);

    kv.key = "RTCM Messages";
    kv.value = std::to_string(rtcm_messages_count_);
    status.values.push_back(kv);

    kv.key = "Reconnect Attempts";
    kv.value = std::to_string(reconnect_attempts_);
    status.values.push_back(kv);

    diag_array->header.stamp = this->now();
    diag_array->status.push_back(status);

    diagnostic_pub_->publish(std::move(diag_array));
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
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    if (fatal)
    {
      is_connected_ = false;
      Disconnect();
    }
  }

  bool NtripClient::ValidateParameters() const
  {
    if (host_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Host must be specified!");
      return false;
    }

    if (port_number_ <= 0 || port_number_ > 65535)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid port number: %d", port_number_);
      return false;
    }

    if (mountpoint_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Mountpoint must be specified!");
      return false;
    }

    if (username_.empty() || password_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Username and password must be specified!");
      return false;
    }

    if (nmea_input_rate_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "NMEA input rate must be positive!");
      return false;
    }

    if (update_rate_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Update rate must be positive!");
      return false;
    }

    if (update_rate_ > nmea_input_rate_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Update rate (%.1f Hz) cannot be higher than NMEA input rate (%.1f Hz)!",
                   update_rate_, nmea_input_rate_);
      return false;
    }

    if (reconnect_delay_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Reconnect delay must be positive!");
      return false;
    }

    return true;
  }

} // namespace ntrip_client
