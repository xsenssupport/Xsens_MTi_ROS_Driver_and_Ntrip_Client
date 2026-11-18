//  Diagnostics publishing for the Xsens MTi ROS 2 driver.
//  Distributed under the same BSD license as the rest of this package.


#include "xsens_diagnostics.h"

#include <xstypes/xsstatusflag.h>

#include <chrono>
#include <cmath>

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
diagnostic_msgs::msg::KeyValue keyValue(const std::string &key, const std::string &value)
{
	diagnostic_msgs::msg::KeyValue kv;
	kv.key = key;
	kv.value = value;
	return kv;
}

std::string toString(double value, int precision = 2)
{
	char buffer[32];
	snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
	return std::string(buffer);
}

std::string boolString(bool value)
{
	return value ? "true" : "false";
}

std::string noRotationString(uint32_t status)
{
	switch (status & XSF_NoRotationMask)
	{
		case 0:
			return "not running";
		case XSF_NoRotationSamplesRejected:
			return "running, samples rejected";
		case XSF_NoRotationAborted:
			return "aborted (device moved)";
		default:
			return "running normally";
	}
}

std::string rtkString(uint32_t status)
{
	switch ((status & XSF_RtkStatus) >> 27)
	{
		case 0:
			return "no RTK";
		case 1:
			return "RTK float";
		case 2:
			return "RTK fixed";
		default:
			return "unknown";
	}
}
}  // namespace

XsensDiagnostics::XsensDiagnostics(DriverNode::SharedPtr node)
	: m_node(node)
{
}

void XsensDiagnostics::declareParameters()
{
	if (!m_node->has_parameter("diagnostics_enabled"))
		m_node->declare_parameter("diagnostics_enabled", true);
	if (!m_node->has_parameter("diagnostics_period"))
		m_node->declare_parameter("diagnostics_period", 1.0);
	if (!m_node->has_parameter("diagnostics_min_rate"))
		m_node->declare_parameter("diagnostics_min_rate", 0.0);
	if (!m_node->has_parameter("diagnostics_stale_timeout"))
		m_node->declare_parameter("diagnostics_stale_timeout", 1.0);
}

void XsensDiagnostics::configure()
{
	declareParameters();

	m_node->get_parameter("diagnostics_enabled", m_enabled);
	m_node->get_parameter("diagnostics_period", m_period);
	m_node->get_parameter("diagnostics_min_rate", m_minRate);
	m_node->get_parameter("diagnostics_stale_timeout", m_staleTimeout);

	if (!m_enabled)
	{
		RCLCPP_INFO(m_node->get_logger(), "Diagnostics publishing is disabled.");
		return;
	}

	if (m_period <= 0.0)
	{
		RCLCPP_WARN(m_node->get_logger(),
					"diagnostics_period must be > 0, got %.3f. Falling back to 1.0 second.", m_period);
		m_period = 1.0;
	}

	m_pub = m_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
	RCLCPP_INFO(m_node->get_logger(), "Publishing diagnostics on /diagnostics every %.2f seconds.", m_period);
}

void XsensDiagnostics::activate()
{
	if (!m_enabled || !m_pub)
		return;

	const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::duration<double>(m_period));

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		// Start the rate window fresh so the first report is not skewed by the
		// time spent inactive.
		m_lastReportTime = m_node->now();
		m_hasReportTime = true;
		m_packetCountAtLastReport = m_packetCount;
		m_errorCountAtLastReport = m_errorCount;
	}

	m_timer = m_node->create_wall_timer(period, [this]() { this->publish(); });
}

void XsensDiagnostics::deactivate()
{
	m_timer.reset();
}

void XsensDiagnostics::cleanup()
{
	m_timer.reset();
	m_pub.reset();
}

void XsensDiagnostics::setDeviceInfo(const std::string &product_code, const std::string &device_id,
									 const std::string &firmware, const std::string &port, int baudrate)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_productCode = product_code;
	m_deviceId = device_id;
	m_firmware = firmware;
	m_portName = port;
	m_baudrate = baudrate;
}

void XsensDiagnostics::setConnected(bool connected)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_connected = connected;
}

void XsensDiagnostics::setMeasuring(bool measuring)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_measuring = measuring;
	if (!measuring)
	{
		// Staleness is only meaningful while the device is streaming.
		m_hasPacketTime = false;
	}
}

void XsensDiagnostics::recordPacket(const XsDataPacket &packet)
{
	const rclcpp::Time now = m_node->now();

	std::lock_guard<std::mutex> lock(m_mutex);
	++m_packetCount;
	m_lastPacketTime = now;
	m_hasPacketTime = true;

	if (packet.containsStatus())
	{
		m_statusWord = packet.status();
		m_hasStatusWord = true;
	}
}

void XsensDiagnostics::recordDeviceError(const std::string &error)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	++m_errorCount;
	m_lastError = error;
}

void XsensDiagnostics::publish()
{
	if (!m_pub)
		return;

	const rclcpp::Time now = m_node->now();

	double rate = 0.0;
	uint32_t errors_in_window = 0;
	uint32_t status_word = 0;
	bool has_status_word = false;

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_hasReportTime)
		{
			const double elapsed = (now - m_lastReportTime).seconds();
			if (elapsed > 0.0)
				rate = static_cast<double>(m_packetCount - m_packetCountAtLastReport) / elapsed;
		}
		m_lastReportTime = now;
		m_hasReportTime = true;
		m_packetCountAtLastReport = m_packetCount;

		errors_in_window = m_errorCount - m_errorCountAtLastReport;
		m_errorCountAtLastReport = m_errorCount;

		status_word = m_statusWord;
		has_status_word = m_hasStatusWord;
	}

	diagnostic_msgs::msg::DiagnosticArray array;
	array.header.stamp = now;

	array.status.push_back(deviceStatus(errors_in_window));
	array.status.push_back(streamStatus(now, rate));
	if (has_status_word)
		array.status.push_back(filterStatus(status_word));

	m_pub->publish(array);
}

diagnostic_msgs::msg::DiagnosticStatus XsensDiagnostics::deviceStatus(uint32_t errors_in_window) const
{
	std::lock_guard<std::mutex> lock(m_mutex);

	DiagnosticStatus status;
	status.name = std::string(m_node->get_name()) + ": Device";
	status.hardware_id = m_deviceId.empty() ? "unknown" : m_deviceId;

	if (!m_connected)
	{
		status.level = DiagnosticStatus::ERROR;
		status.message = "No device connected";
	}
	else if (errors_in_window > 0)
	{
		status.level = DiagnosticStatus::WARN;
		status.message = "Device reported an error: " + m_lastError;
	}
	else
	{
		status.level = DiagnosticStatus::OK;
		status.message = m_measuring ? "Connected, measuring" : "Connected, not measuring";
	}

	status.values.push_back(keyValue("Product code", m_productCode));
	status.values.push_back(keyValue("Device ID", m_deviceId));
	status.values.push_back(keyValue("Firmware version", m_firmware));
	status.values.push_back(keyValue("Port", m_portName));
	status.values.push_back(keyValue("Baudrate", std::to_string(m_baudrate)));
	status.values.push_back(keyValue("Measuring", boolString(m_measuring)));
	status.values.push_back(keyValue("Device errors (total)", std::to_string(m_errorCount)));
	if (!m_lastError.empty())
		status.values.push_back(keyValue("Last device error", m_lastError));

	return status;
}

diagnostic_msgs::msg::DiagnosticStatus XsensDiagnostics::streamStatus(const rclcpp::Time &now, double rate)
{
	std::lock_guard<std::mutex> lock(m_mutex);

	DiagnosticStatus status;
	status.name = std::string(m_node->get_name()) + ": Data stream";
	status.hardware_id = m_deviceId.empty() ? "unknown" : m_deviceId;

	const double age = m_hasPacketTime ? (now - m_lastPacketTime).seconds() : -1.0;

	if (!m_measuring)
	{
		status.level = DiagnosticStatus::OK;
		status.message = "Idle, device is not in measurement mode";
	}
	else if (!m_hasPacketTime)
	{
		status.level = DiagnosticStatus::ERROR;
		status.message = "Measuring but no data received yet";
	}
	else if (age > m_staleTimeout)
	{
		status.level = DiagnosticStatus::STALE;
		status.message = "No data for " + toString(age) + " s";
	}
	else if (m_minRate > 0.0 && rate < m_minRate)
	{
		status.level = DiagnosticStatus::WARN;
		status.message = "Data rate " + toString(rate, 1) + " Hz is below the configured minimum of " +
						 toString(m_minRate, 1) + " Hz";
	}
	else
	{
		status.level = DiagnosticStatus::OK;
		status.message = "Streaming at " + toString(rate, 1) + " Hz";
	}

	status.values.push_back(keyValue("Packets received", std::to_string(m_packetCount)));
	status.values.push_back(keyValue("Rate (Hz)", toString(rate, 1)));
	status.values.push_back(keyValue("Minimum rate (Hz)", m_minRate > 0.0 ? toString(m_minRate, 1) : "not checked"));
	status.values.push_back(keyValue("Time since last packet (s)", age < 0.0 ? "n/a" : toString(age, 3)));
	status.values.push_back(keyValue("Stale timeout (s)", toString(m_staleTimeout)));

	return status;
}

diagnostic_msgs::msg::DiagnosticStatus XsensDiagnostics::filterStatus(uint32_t status_word) const
{
	const bool self_test_ok = (status_word & XSF_SelfTestOk) != 0;
	const bool orientation_valid = (status_word & XSF_OrientationValid) != 0;
	const bool gnss_fix = (status_word & XSF_GpsValid) != 0;
	const bool clipping = (status_word & XSF_ClippingDetected) != 0;
	const bool clock_synced = (status_word & XSF_ExternalClockSynced) != 0;
	const bool gnss_time_pulse = (status_word & XSF_HaveGnssTimePulse) != 0;

	DiagnosticStatus status;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		status.name = std::string(m_node->get_name()) + ": Filter status";
		status.hardware_id = m_deviceId.empty() ? "unknown" : m_deviceId;
	}

	// The self test flag is only set right after an explicit self test command,
	// so it is reported as a value but never drives the level: a healthy MTi
	// leaves it clear during normal measurement.
	if (!orientation_valid)
	{
		status.level = DiagnosticStatus::WARN;
		status.message = "Orientation is not valid";
	}
	else if (clipping)
	{
		status.level = DiagnosticStatus::WARN;
		status.message = "Sensor data is clipping";
	}
	else
	{
		status.level = DiagnosticStatus::OK;
		status.message = "Filter is running";
	}

	char status_hex[16];
	snprintf(status_hex, sizeof(status_hex), "0x%08X", status_word);

	status.values.push_back(keyValue("Status word", status_hex));
	status.values.push_back(keyValue("Self test ok", boolString(self_test_ok)));
	status.values.push_back(keyValue("Orientation valid", boolString(orientation_valid)));
	status.values.push_back(keyValue("GNSS fix", boolString(gnss_fix)));
	status.values.push_back(keyValue("RTK status", rtkString(status_word)));
	status.values.push_back(keyValue("No rotation update", noRotationString(status_word)));
	status.values.push_back(keyValue("Clipping detected", boolString(clipping)));
	status.values.push_back(keyValue("Clipping acc", boolString(anyAccClipped(static_cast<int>(status_word)))));
	status.values.push_back(keyValue("Clipping gyr", boolString(anyGyrClipped(static_cast<int>(status_word)))));
	status.values.push_back(keyValue("Clipping mag", boolString(anyMagClipped(static_cast<int>(status_word)))));
	status.values.push_back(keyValue("Filter mode", std::to_string((status_word & XSF_FilterMode) >> 23)));
	status.values.push_back(keyValue("External clock synced", boolString(clock_synced)));
	status.values.push_back(keyValue("GNSS time pulse", boolString(gnss_time_pulse)));

	return status;
}
