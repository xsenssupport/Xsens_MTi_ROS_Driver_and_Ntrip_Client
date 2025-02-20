#include "ntrip_client/ntrip_client.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ntrip_client::NtripClient>();
  
  if (!node->Start()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start NTRIP client");
    return 1;
  }

  rclcpp::spin(node);
  
  node->Stop();
  rclcpp::shutdown();
  
  return 0;
}