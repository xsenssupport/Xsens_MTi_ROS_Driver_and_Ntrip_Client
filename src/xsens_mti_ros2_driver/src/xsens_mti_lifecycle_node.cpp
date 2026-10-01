//  Managed (lifecycle) node wrapper for the Xsens MTi ROS 2 driver.
//  Distributed under the same BSD license as the rest of this package.


#include "xsens_mti_lifecycle_node.h"

#include "xdainterface.h"
#include "xsens_diagnostics.h"

using CallbackReturn = XsensMtiLifecycleNode::CallbackReturn;

XsensMtiLifecycleNode::XsensMtiLifecycleNode(const rclcpp::NodeOptions &options)
	: DriverNode("xsens_driver", options)
{
	if (!has_parameter("autostart"))
		declare_parameter("autostart", true);
}

XsensMtiLifecycleNode::~XsensMtiLifecycleNode()
{
	releaseDevice();
}

bool XsensMtiLifecycleNode::autostart() const
{
	bool autostart = true;
	get_parameter("autostart", autostart);
	return autostart;
}

bool XsensMtiLifecycleNode::isActive() const
{
	return m_active;
}

void XsensMtiLifecycleNode::pumpDeviceData(std::chrono::milliseconds timeout)
{
	if (m_active && m_interface)
		m_interface->spinFor(timeout);
}

void XsensMtiLifecycleNode::shutdownDriver()
{
	if (m_active && m_interface)
		m_interface->stopMeasurement();

	releaseDevice();
}

void XsensMtiLifecycleNode::releaseDevice()
{
	m_active = false;

	// Destroying the interface closes the log file, releases the port and
	// destroys the message publishers.
	m_interface.reset();

	if (m_diagnostics)
	{
		m_diagnostics->cleanup();
		m_diagnostics.reset();
	}
}

CallbackReturn XsensMtiLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Configuring ...");

	m_diagnostics = std::make_shared<XsensDiagnostics>(shared_from_this());
	m_diagnostics->configure();

	m_interface = std::make_shared<XdaInterface>(shared_from_this());
	m_interface->setDiagnostics(m_diagnostics);

	if (!m_interface->connectDevice())
	{
		RCLCPP_ERROR(get_logger(), "Failed to connect device");
		releaseDevice();
		return CallbackReturn::FAILURE;
	}

	// Publishers are created here and stay deactivated until the node is
	// activated, so no data reaches the wire before then.
	m_interface->registerPublishers();

	if (!m_interface->configureDevice())
	{
		RCLCPP_ERROR(get_logger(), "Failed to configure device");
		releaseDevice();
		return CallbackReturn::FAILURE;
	}

	RCLCPP_INFO(get_logger(), "Configured.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn XsensMtiLifecycleNode::on_activate(const rclcpp_lifecycle::State &previous_state)
{
	RCLCPP_INFO(get_logger(), "Activating ...");

	// Activates every lifecycle publisher created during configuration.
	DriverNode::on_activate(previous_state);

	if (!m_interface || !m_interface->startMeasurement())
	{
		RCLCPP_ERROR(get_logger(), "Failed to put the device into measurement mode");
		DriverNode::on_deactivate(previous_state);
		return CallbackReturn::FAILURE;
	}

	m_diagnostics->activate();
	m_active = true;

	RCLCPP_INFO(get_logger(), "Activated.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn XsensMtiLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
	RCLCPP_INFO(get_logger(), "Deactivating ...");

	m_active = false;

	if (m_diagnostics)
		m_diagnostics->deactivate();
	if (m_interface)
		m_interface->stopMeasurement();

	// Deactivates every lifecycle publisher created during configuration.
	DriverNode::on_deactivate(previous_state);

	RCLCPP_INFO(get_logger(), "Deactivated.");
	return CallbackReturn::SUCCESS;
}

CallbackReturn XsensMtiLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Cleaning up ...");
	releaseDevice();
	return CallbackReturn::SUCCESS;
}

CallbackReturn XsensMtiLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
	RCLCPP_INFO(get_logger(), "Shutting down ...");

	// Leaves the publishers deactivated if we were shut down while active.
	DriverNode::on_deactivate(previous_state);

	shutdownDriver();

	return CallbackReturn::SUCCESS;
}

CallbackReturn XsensMtiLifecycleNode::on_error(const rclcpp_lifecycle::State &)
{
	RCLCPP_ERROR(get_logger(), "A lifecycle transition failed, releasing the device.");
	releaseDevice();
	return CallbackReturn::SUCCESS;
}
