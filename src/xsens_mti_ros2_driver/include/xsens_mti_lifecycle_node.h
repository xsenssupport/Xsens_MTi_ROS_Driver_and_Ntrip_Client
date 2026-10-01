//  Managed (lifecycle) node wrapper for the Xsens MTi ROS 2 driver.
//  Distributed under the same BSD license as the rest of this package.


#ifndef XSENS_MTI_LIFECYCLE_NODE_H
#define XSENS_MTI_LIFECYCLE_NODE_H

#include <chrono>
#include <memory>

#include "xsens_driver_types.h"

class XdaInterface;
class XsensDiagnostics;

//! \brief The driver as a managed node.
//!
//! The lifecycle states map onto the device as follows:
//!  - configuring:  open the port, create the publishers, write the device configuration
//!  - activating:   put the device into measurement mode, start diagnostics reporting
//!  - deactivating: put the device back into config mode, stop publishing
//!  - cleaning up:  close the port and destroy the publishers
class XsensMtiLifecycleNode : public DriverNode
{
public:
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

	explicit XsensMtiLifecycleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	~XsensMtiLifecycleNode() override;

	//! \brief Whether the node should walk itself into the active state on startup.
	bool autostart() const;
	//! \brief Whether the device is streaming and the publishers are activated.
	bool isActive() const;

	//! \brief Hand any pending device data to the message publishers.
	//! Does nothing unless the node is active.
	void pumpDeviceData(std::chrono::milliseconds timeout);

	//! \brief Stop the device and release everything the node holds.
	//!
	//! The driver objects owned by this node hold a shared reference back to it,
	//! so the node cannot destroy itself while they are alive. Call this before
	//! dropping the last reference to the node (the lifecycle 'shutdown'
	//! transition does it too) so the port is closed and recording is stopped.
	void shutdownDriver();

	CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
	CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
	void releaseDevice();

	std::shared_ptr<XdaInterface> m_interface;
	std::shared_ptr<XsensDiagnostics> m_diagnostics;
	bool m_active = false;
};

#endif
