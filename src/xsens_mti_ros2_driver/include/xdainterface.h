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


#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/rtcm.hpp>
#include "xdacallback.h"
#include <xstypes/xsportinfo.h>
#include <xstypes/xsstring.h>

#include <chrono>

struct XsControl;
struct XsDevice;
struct XsString;
struct XsPortInfo;

class PacketCallback;

class XdaInterface
{
public:
	explicit XdaInterface(rclcpp::Node::SharedPtr node);
	~XdaInterface();

	void spinFor(std::chrono::milliseconds timeout);
	void registerPublishers();
	void rtcmCallback(const mavros_msgs::msg::RTCM::SharedPtr msg);

	bool connectDevice();
	bool prepare();
	void close();

	void setupManualGyroBiasEstimation();

private:
	void registerCallback(PacketCallback *cb);
	bool handleError(std::string error);
	void declareCommonParameters();
	bool configureSensorSettings();
	bool manualGyroBiasEstimation(uint16_t duration);

	XsControl *m_control;
	XsDevice *m_device;
	XsString m_productCode;
	XsPortInfo m_port;
	XdaCallback m_xdaCallback;
	std::list<PacketCallback *> m_callbacks;
	rclcpp::Node::SharedPtr m_node; 
	// Timer for Manual Gyro Bias Estimation
	rclcpp::TimerBase::SharedPtr m_manualGyroBiasTimer;
	rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr m_rtcmSubscription;
};

#endif