//  Diagnostics publishing for the Xsens MTi ROS 2 driver.
//  Distributed under the same BSD license as the rest of this package.


#ifndef XSENS_DIAGNOSTICS_H
#define XSENS_DIAGNOSTICS_H

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <xstypes/xsdatapacket.h>

#include <mutex>
#include <string>

#include "xsens_driver_types.h"

//! \brief Collects driver health information and publishes it on /diagnostics.
//!
//! Three statuses are published: the state of the device connection, the health
//! of the measurement stream (rate, staleness) and a decoded view of the last
//! MTi status word. Sampling methods are safe to call from the XDA callback
//! thread; publishing happens on the node's executor thread.
class XsensDiagnostics
{
public:
	explicit XsensDiagnostics(DriverNode::SharedPtr node);

	//! \brief Declare the diagnostics parameters on the node.
	void declareParameters();

	//! \brief Create the /diagnostics publisher. Called from on_configure().
	void configure();
	//! \brief Start the periodic publish timer. Called from on_activate().
	void activate();
	//! \brief Stop the periodic publish timer. Called from on_deactivate().
	void deactivate();
	//! \brief Release publisher and timer. Called from on_cleanup().
	void cleanup();

	//! \brief Whether diagnostics publishing is enabled by parameter.
	bool enabled() const
	{
		return m_enabled;
	}

	void setDeviceInfo(const std::string &product_code, const std::string &device_id,
					   const std::string &firmware, const std::string &port, int baudrate);
	void setConnected(bool connected);
	void setMeasuring(bool measuring);

	//! \brief Account for one packet handed to the message publishers.
	void recordPacket(const XsDataPacket &packet);
	//! \brief Account for one error reported by the device.
	void recordDeviceError(const std::string &error);

	//! \brief Assemble and publish a DiagnosticArray. Normally driven by a timer.
	void publish();

private:
	diagnostic_msgs::msg::DiagnosticStatus deviceStatus(uint32_t errors_in_window) const;
	diagnostic_msgs::msg::DiagnosticStatus streamStatus(const rclcpp::Time &now, double rate);
	diagnostic_msgs::msg::DiagnosticStatus filterStatus(uint32_t status_word) const;

	DriverNode::SharedPtr m_node;
	DriverPublisher<diagnostic_msgs::msg::DiagnosticArray> m_pub;
	rclcpp::TimerBase::SharedPtr m_timer;

	// Parameters
	bool m_enabled = true;
	double m_period = 1.0;
	double m_minRate = 0.0;
	double m_staleTimeout = 1.0;

	mutable std::mutex m_mutex;
	std::string m_productCode;
	std::string m_deviceId;
	std::string m_firmware;
	std::string m_portName;
	int m_baudrate = 0;
	bool m_connected = false;
	bool m_measuring = false;

	uint64_t m_packetCount = 0;
	uint64_t m_packetCountAtLastReport = 0;
	rclcpp::Time m_lastPacketTime;
	bool m_hasPacketTime = false;

	uint32_t m_statusWord = 0;
	bool m_hasStatusWord = false;

	uint32_t m_errorCount = 0;
	uint32_t m_errorCountAtLastReport = 0;
	std::string m_lastError;

	rclcpp::Time m_lastReportTime;
	bool m_hasReportTime = false;
};

#endif
