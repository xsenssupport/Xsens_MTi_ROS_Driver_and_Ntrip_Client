#include <unistd.h>
#include <stdint.h>
#include <string>
#include <vector>

#include "ntrip_client.h"
#include "ntrip_util.h"
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/rtcm.hpp>
#include <std_msgs/msg/header.hpp>
#include <nmea_msgs/msg/sentence.hpp>

using libntrip::NtripClient;
using namespace std::chrono_literals;

std::string ip, user, passwd, mountpoint, rtcm_topic;
int port = 8001;
double longitude = 0, latitude = 0, height = 0;
nmea_msgs::msg::Sentence nmea_msg;
NtripClient ntrip_client;

void gnssCallback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
    nmea_msg = *msg;
    ntrip_client.set_gga_buffer(nmea_msg.sentence);
    ntrip_client.SetGnssDataReceived(true);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ntrip_client");
    
    int report_interval = 1; // default value of 1 second frequency sending to NTRIP Caster


    node->declare_parameter("ip", "");
    node->declare_parameter("port", 8001);
    node->declare_parameter("user", "");
    node->declare_parameter("passwd", "");
    node->declare_parameter("mountpoint", "");
    node->declare_parameter("rtcm_topic", "/rtcm");
    node->declare_parameter("report_interval", 1);


    node->get_parameter("ip", ip);
    node->get_parameter("port", port);
    node->get_parameter("user", user);
    node->get_parameter("passwd", passwd);
    node->get_parameter("mountpoint", mountpoint);
    node->get_parameter("rtcm_topic", rtcm_topic);
    node->get_parameter("report_interval", report_interval);


    auto pubRTCM = node->create_publisher<mavros_msgs::msg::RTCM>(rtcm_topic, 10);
    auto sub = node->create_subscription<nmea_msgs::msg::Sentence>("/nmea", 100, gnssCallback);

    int seq = 0;
    ntrip_client.Init(ip, port, user, passwd, mountpoint);

    ntrip_client.OnReceived([&](const char *buffer, int size)
                            {
                                std::vector<uint8_t> data(size);
                                for (int i = 0; i < size; i++)
                                {
                                    data[i] = static_cast<uint8_t>(buffer[i]);
                                }

                                rclcpp::Time stamp = node->now();
                                std_msgs::msg::Header header;
                                header.frame_id = "rtcm";
                                header.stamp = stamp;
                                mavros_msgs::msg::RTCM rmsg;
                                rmsg.header = header;
                                rmsg.data = data;
                                pubRTCM->publish(rmsg);
                            });

    ntrip_client.set_report_interval(report_interval);

    ntrip_client.Run();
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Maybe take longer?

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && ntrip_client.service_is_running())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    ntrip_client.Stop();
    rclcpp::shutdown();
    return 0;
}

