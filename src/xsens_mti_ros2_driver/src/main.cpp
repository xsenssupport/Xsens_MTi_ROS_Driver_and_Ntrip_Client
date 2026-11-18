//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  



#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "xsens_mti_lifecycle_node.h"
#include <xscommon/journaller.h>
#include <mavros_msgs/msg/rtcm.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <atomic>
#include <csignal>

using std::chrono::milliseconds;
using lifecycle_msgs::msg::State;

Journaller *gJournal = 0;

namespace
{
std::atomic_bool g_keep_running{true};

void requestStop(int)
{
    g_keep_running = false;
}
}  // namespace

int main(int argc, char *argv[])
{
    // Handle the termination signals ourselves so that the lifecycle 'shutdown'
    // transition still runs on a valid context, which lets the node leave
    // measurement mode and close the port in an orderly way.
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With SingleThreadedExecutor, all callbacks will be called from within this thread (the main thread in this case).
    rclcpp::executors::SingleThreadedExecutor exec;

    // Create the managed node called "xsens_driver"
    auto node = std::make_shared<XsensMtiLifecycleNode>();
    // Add the node to the executor
    exec.add_node(node->get_node_base_interface());

    // Unless the node is driven externally (autostart:=false), walk it into the
    // active state right away so that the node behaves as it always has.
    if (node->autostart())
    {
        if (node->configure().id() != State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to configure the driver");
            node->shutdownDriver();
            rclcpp::shutdown();
            return -1;
        }

        if (node->activate().id() != State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to activate the driver");
            node->shutdownDriver();
            rclcpp::shutdown();
            return -1;
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),
                    "Started with autostart:=false. Waiting for lifecycle transitions on ~/change_state.");
    }

    while (rclcpp::ok() && g_keep_running)
    {
        if (node->isActive())
        {
            // Blocks until a packet arrives or the timeout expires.
            node->pumpDeviceData(milliseconds(100));
            exec.spin_some();
        }
        else
        {
            // Nothing to read from the device, so just wait for lifecycle
            // service calls instead of spinning hot.
            exec.spin_once(milliseconds(100));
        }
    }

    // Walk the state machine to 'finalized' so that the node is not destroyed
    // while it is still active.
    if (rclcpp::ok() && node->get_current_state().id() != State::PRIMARY_STATE_FINALIZED)
        node->shutdown();

    // Release the device before dropping the last reference to the node.
    node->shutdownDriver();

    rclcpp::shutdown();

    return 0;
}
