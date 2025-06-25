//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright
//  notice, 	this list of conditions, and the following disclaimer in the
//  documentation 	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their
//  contributors 	may be used to endorse or promote products derived from
//  this software without 	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS SHALL BE EXCLUSIVELY
//  APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES OF
//  ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR
//  MORE ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <iostream>
#include <mavros_msgs/msg/rtcm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <string>

#include "xdainterface.h"

using std::chrono::milliseconds;

Journaller *gJournal = 0;

class ImuNodeWatchdog {
 private:
  // The XdaInterface instance will handle the connection to the Xsens device
  std::shared_ptr<XdaInterface> xdaInterface_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr error_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Watchdog variables
  rclcpp::Time last_info_message_time_;
  rclcpp::Time last_message_time_;
  std::chrono::milliseconds timeout_duration_;
  int driver_fault_counter_ = 0;
  bool published_driver_error_ = false;

  const rclcpp::Duration thres0_ = rclcpp::Duration(std::chrono::milliseconds(10'000));
  const rclcpp::Duration thres1_ = rclcpp::Duration(std::chrono::milliseconds(500));
  const rclcpp::Duration thres2_ = rclcpp::Duration(std::chrono::milliseconds(5000));

  std::mutex mutex_;

  bool resetAndInitXdaInterface() {
    // Declare the XdaInterface with the node
    xdaInterface_ = std::make_shared<XdaInterface>(node_);
    RCLCPP_INFO(node_->get_logger(), "XdaInterface has been initialized");

    if (!xdaInterface_->connectDevice()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect device");
      return false;
    }

    xdaInterface_->registerPublishers();

    if (!xdaInterface_->prepare()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to prepare device");
      return false;
    }

    // Reset the xdaInterface pointer to ensure it is destroyed before calling
    // rclcpp::shutdown()
    xdaInterface_.reset();
    return true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    mutex_.lock();
    last_message_time_ = rclcpp::Time(msg->header.stamp);
    mutex_.unlock();
  }

  void check_watchdog() {
    auto current_time = node_->now();
    mutex_.lock();
    auto time_since_last_msg = current_time - last_message_time_;
    mutex_.unlock();

    auto time_since_last_info_msg = current_time - last_info_message_time_;
    if (time_since_last_info_msg > thres0_) {
      std::cerr << "IMU-WATCHDOG: IMU healthy (" << current_time.seconds() << ")\n";
      last_info_message_time_ = current_time;
    }

    if (time_since_last_msg > thres1_) {
      std::cerr << "IMU-WATCHDOG: IMU driver fault detected! No messages received for "
                << time_since_last_msg.seconds() << " seconds. Restarting driver...\n";
      resetAndInitXdaInterface();

      if (resetAndInitXdaInterface()) {
        std::cerr << "XdaInterface successfully reinitialized.\n";
      } else {
        driver_fault_counter_++;
        std::cerr << "Failed to reinitialize XdaInterface. Driver fault counter: "
                  << driver_fault_counter_ << "\n";
      }
    }

    if (!published_driver_error_ && time_since_last_msg > thres2_) {
      std::cerr << "IMU-WATCHDOG: IMU driver fault detected! No messages received for "
                << time_since_last_msg.seconds() << " seconds. Sending error message to GUI...\n";

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "IMU Driver Fault";
      status.message = "IMU driver failed, please stop the scan and start a new one.";
      status.level = status.ERROR;

      diagnostic_msgs::msg::DiagnosticArray errors;
      errors.header.stamp = current_time;
      errors.status.push_back(status);
      error_pub_->publish(errors);

      published_driver_error_ = true; // only publish once
    }

  }

 public:
  ImuNodeWatchdog() {
    // Create an executor that will be responsible for execution of callbacks
    // for a set of nodes. With SingleThreadedExecutor, all callbacks will be
    // called from within this thread (the main thread in this case).
    rclcpp::executors::SingleThreadedExecutor exec;
    node_ = std::make_shared<rclcpp::Node>("xsens_driver");
    exec.add_node(node_);

    if (!resetAndInitXdaInterface()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize XdaInterface");
      return;
    }

    // Subscriber to monitor our own published messages
    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&ImuNodeWatchdog::imu_callback, this, std::placeholders::_1));
    watchdog_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),  // Check every 100ms
                                               std::bind(&ImuNodeWatchdog::check_watchdog, this));
    error_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/errors", 1);

    while (rclcpp::ok()) {
      xdaInterface_->spinFor(milliseconds(100));
      exec.spin_some();
    }
  }

  ~ImuNodeWatchdog() {
    std::cerr << "Shutting down ImuNodeWatchdog\n";
    rclcpp::shutdown();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ImuNodeWatchdog watchdog;
  return 0;
}
