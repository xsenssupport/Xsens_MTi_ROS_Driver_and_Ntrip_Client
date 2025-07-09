#include "ntrip_client/ntrip_client_lifecycle.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn ntrip_client::NtripClientLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");
  DeclareParameters();
  if (!Initialize()) return CallbackReturn::FAILURE;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ntrip_client::NtripClientLifecycle::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");
  Start();
  rtcm_pub_->on_activate();
  diagnostic_pub_->on_activate();
  connection_timer_->reset();
  status_timer_->reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ntrip_client::NtripClientLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  Stop();
  rtcm_pub_->on_deactivate();
  diagnostic_pub_->on_deactivate();
  connection_timer_->cancel();
  status_timer_->cancel();
  return CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ntrip_client::NtripClientLifecycle>();
  
    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();
    
    return 0;
  }
