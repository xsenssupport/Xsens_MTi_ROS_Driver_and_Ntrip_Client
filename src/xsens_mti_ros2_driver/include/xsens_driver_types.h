//  Shared node/publisher type aliases for the Xsens MTi ROS 2 driver.
//  Distributed under the same BSD license as the rest of this package.


#ifndef XSENS_DRIVER_TYPES_H
#define XSENS_DRIVER_TYPES_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

// The driver runs as a managed (lifecycle) node. Publishers created through it
// are lifecycle publishers, which only put messages on the wire while the node
// is in the 'active' state.
using DriverNode = rclcpp_lifecycle::LifecycleNode;

template <typename MessageT>
using DriverPublisher = typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr;

#endif
