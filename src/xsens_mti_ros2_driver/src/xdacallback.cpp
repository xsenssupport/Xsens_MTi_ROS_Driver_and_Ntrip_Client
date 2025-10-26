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


#include "xdacallback.h"

#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>

XdaCallback::XdaCallback(rclcpp::Node::SharedPtr node, size_t maxBufferSize)
	: m_maxBufferSize(maxBufferSize)
	, parent_node(node)
	, m_interpolator(node)
	, m_interpolationEnabled(false)
{
	int time_option = 0; //default is "mti_utc"
	parent_node->declare_parameter<int>("time_option", 0);
	parent_node->get_parameter("time_option", time_option);
	m_timeHandler.setTimeOption(time_option);
	//if else to check time_option rosinfo to print time_option
	if (time_option == 0)
	{
		RCLCPP_INFO(parent_node->get_logger(), "Rosnode time_option parameter is utc time from MTi");
	}
	else if (time_option == 1)
	{
		RCLCPP_INFO(parent_node->get_logger(), "Rosnode time_option parameter is sample time fine from MTi");
	}
	else
	{
		RCLCPP_WARN(parent_node->get_logger(), "Rosnode time_option parameter is using host controller's ros time. ");
	}

	// Declare and get interpolation parameter
	parent_node->declare_parameter<bool>("interpolate_orientation_high_rate", false);
	parent_node->get_parameter("interpolate_orientation_high_rate", m_interpolationEnabled);
	
	if (m_interpolationEnabled)
	{
		RCLCPP_INFO(parent_node->get_logger(), "Rosnode interpolate_orientation_high_rate parameter is enabled");
	}
	else
	{
		RCLCPP_INFO(parent_node->get_logger(), "Rosnode interpolate_orientation_high_rate parameter is disabled");
	}
}

XdaCallback::~XdaCallback() throw()
{
}

// Returns empty packet on timeout
RosXsDataPacket XdaCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_condition.wait_for(lock, timeout, [&] { return !m_buffer.empty(); }))
	{
		assert(!m_buffer.empty());

		packet = m_buffer.front();
		m_buffer.pop_front();
	}

	return packet;
}

void XdaCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
	std::unique_lock<std::mutex> lock(m_mutex);

	assert(packet != 0);

	// Check if interpolation is enabled
	if (m_interpolationEnabled)
	{
		// Process packet for interpolation
		XsDataPacket interpolatedPacket;
		bool interpolated = m_interpolator.processPacket(*packet, interpolatedPacket);
		
		// Only buffer interpolated packets (which contain all three data types)
		if (interpolated)
		{
			// Discard oldest packet if buffer full
			if (m_buffer.size() == m_maxBufferSize)
			{
				m_buffer.pop_front();
			}

			rclcpp::Time now = m_timeHandler.convertUtcTimeToRosTime(interpolatedPacket);
			// Push interpolated packet
			m_buffer.push_back(RosXsDataPacket(now, interpolatedPacket));

			// Manual unlocking is done before notifying, to avoid waking up
			// the waiting thread only to block again
			lock.unlock();
			m_condition.notify_one();
		}
		// If not interpolated yet (still buffering), don't push to output buffer
	}
	else
	{
		// Interpolation disabled - pass through all packets as-is
		// Discard oldest packet if buffer full
		if (m_buffer.size() == m_maxBufferSize)
		{
			m_buffer.pop_front();
		}

		rclcpp::Time now = m_timeHandler.convertUtcTimeToRosTime(*packet);
		// Push original packet
		m_buffer.push_back(RosXsDataPacket(now, *packet));

		// Manual unlocking is done before notifying, to avoid waking up
		// the waiting thread only to block again
		lock.unlock();
		m_condition.notify_one();
	}
}

void XdaCallback::onError(XsDevice *dev, XsResultValue error)
{
	RCLCPP_ERROR(parent_node->get_logger(), "MTi Error: %s", XsResultValue_toString(error));
	if(error == XRV_DATAOVERFLOW)
	{
		RCLCPP_ERROR(parent_node->get_logger(), "Data overflow occurred. Use MT Manager - Device Settings, to change the baudrate to higher value like 921600 or 2000000!! Optionally, change the enable_outputConfig to true to change the output in the xsens_mti_node.yaml. If both doesn't work, reduce your output data rate.");
	}
}
