// Distributed under the BSD license of the Xsens MTi ROS 2 driver.
#ifndef XSENS_SERIAL_LOW_LATENCY_H
#define XSENS_SERIAL_LOW_LATENCY_H

#include <rclcpp/rclcpp.hpp>
#include <string>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <linux/tty_flags.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace xsens {

// Requests ASYNC_LOW_LATENCY on a serial port.
//
// On FTDI-based adapters (ftdi_sio), which is how most MTi development boards
// and USB cables enumerate, this reduces the driver's latency timer from its
// 16 ms default to 1 ms. The 16 ms default makes the kernel deliver data in
// bursts, which adds latency and can overflow the XdaCallback buffer at high
// output rates.
//
// The flag is a property of the port, not of this file descriptor, so it stays
// in effect after the descriptor is closed and applies to the handle that XDA
// opens afterwards. It is reset when the device is unplugged.
//
// Requires read/write access to the port, which membership of the 'dialout'
// group provides; root is not needed. Returns false and logs a warning when
// unsupported, so callers can treat this as a best-effort optimisation.
inline bool setSerialLowLatency(const std::string &port, const rclcpp::Logger &logger)
{
#ifndef __linux__
	RCLCPP_WARN(logger, "enable_low_latency is only supported on Linux; ignoring it for port %s", port.c_str());
	return false;
#else
	const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
	{
		RCLCPP_WARN(logger, "Could not open %s to set low latency: %s", port.c_str(), strerror(errno));
		return false;
	}

	struct serial_struct serinfo;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
	{
		// Not every tty driver implements TIOCGSERIAL, so this is not an error.
		RCLCPP_WARN(logger, "TIOCGSERIAL failed on %s: %s. Low latency not applied.", port.c_str(), strerror(errno));
		close(fd);
		return false;
	}

	if (serinfo.flags & ASYNC_LOW_LATENCY)
	{
		RCLCPP_INFO(logger, "%s is already in low latency mode.", port.c_str());
		close(fd);
		return true;
	}

	serinfo.flags |= ASYNC_LOW_LATENCY;
	if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
	{
		RCLCPP_WARN(logger, "TIOCSSERIAL failed on %s: %s. Low latency not applied.", port.c_str(), strerror(errno));
		close(fd);
		return false;
	}

	close(fd);
	RCLCPP_INFO(logger, "Set %s to low latency mode.", port.c_str());
	return true;
#endif
}

}
#endif
