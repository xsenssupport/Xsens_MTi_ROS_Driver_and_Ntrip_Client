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
#include "xdainterface.h"
#include <mavros_msgs/msg/rtcm.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>

using std::chrono::milliseconds;

Journaller *gJournal = 0;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With SingleThreadedExecutor, all callbacks will be called from within this thread (the main thread in this case).
    rclcpp::executors::SingleThreadedExecutor exec;

    // Create a node called "xsens_driver"
    auto node = std::make_shared<rclcpp::Node>("xsens_driver");
    // Add the node to the executor
    exec.add_node(node);

    // Declare the XdaInterface with the node
    auto xdaInterface = std::make_shared<XdaInterface>(node);
    RCLCPP_INFO(node->get_logger(), "XdaInterface has been initialized");

    if (!xdaInterface->connectDevice()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect device");
        return -1;
    }

    xdaInterface->registerPublishers();

    if (!xdaInterface->prepare()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to prepare device");
        return -1;
    }


    while (rclcpp::ok())
    {
        xdaInterface->spinFor(milliseconds(100));
        exec.spin_some();
    }

    // Reset the xdaInterface pointer to ensure it is destroyed before calling rclcpp::shutdown()
    xdaInterface.reset();

    rclcpp::shutdown();

    return 0;
}
