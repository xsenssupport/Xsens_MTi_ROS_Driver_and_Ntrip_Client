#include <unistd.h>
#include <stdint.h>
#include <string>
#include <vector>

#include "ntrip_client.h"
#include "ntrip_util.h"
#include <ros/ros.h>
#include <mavros_msgs/RTCM.h>
#include <std_msgs/Header.h>
#include <nmea_msgs/Sentence.h>

using libntrip::NtripClient;

std::string ip, user, passwd, mountpoint, rtcm_topic;
int port = 8001;
double longitude = 0, latitude = 0, height = 0;
nmea_msgs::Sentence nmea_msg;
NtripClient ntrip_client; // Make ntrip_client a global variable
int report_interval = 1; // default value of 1 second frequency sending to NTRIP Caster

//define callback function gnssCallback to save the lat, long, height
void gnssCallback(const nmea_msgs::Sentence &msg)
{
    nmea_msg = msg;
    ntrip_client.set_gga_buffer(nmea_msg.sentence);
    ntrip_client.SetGnssDataReceived(true); // Add this line to indicate that GNSS data has been received
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ntrip_client");
    ros::NodeHandle nh;

    if (!nh.getParam("/ntrip_node/ip", ip))
    {
        ROS_ERROR("Failed to get 'ip' parameter");
        return -1; // Exit if critical parameter not found
    }
    if (!nh.getParam("/ntrip_node/port", port))
    {
        ROS_ERROR("Failed to get 'port' parameter");
        return -1;
    }
    if (!nh.getParam("/ntrip_node/user", user))
    {
        ROS_ERROR("Failed to get 'user' parameter");
        return -1;
    }
    if (!nh.getParam("/ntrip_node/passwd", passwd))
    {
        ROS_ERROR("Failed to get 'passwd' parameter");
        return -1;
    }
    if (!nh.getParam("/ntrip_node/mountpoint", mountpoint))
    {
        ROS_ERROR("Failed to get 'mountpoint' parameter");
        return -1;
    }
    if (!nh.getParam("/ntrip_node/report_interval", report_interval))
    {
        report_interval = 1; // Set default value if parameter not found
    }

    ros::Publisher pubRTCM = nh.advertise<mavros_msgs::RTCM>("/rtcm", 10);
    ros::Subscriber sub = nh.subscribe("/nmea", 100, gnssCallback);

    int seq = 0;
    ntrip_client.Init(ip, port, user, passwd, mountpoint);
    ntrip_client.OnReceived([&](const char *buffer, int size)
                            {
                                std::vector<uint8_t> data(size);
                                for (int i = 0; i < size; i++)
                                {
                                    data[i] = static_cast<uint8_t>(buffer[i]);
                                }
                                
                                ros::Time stamp = ros::Time::now();
                                std_msgs::Header header;
                                header.frame_id = "rtcm";
                                header.seq = seq++;
                                header.stamp = stamp;
                                mavros_msgs::RTCM rmsg;
                                rmsg.header = header;
                                rmsg.data = data;
                                pubRTCM.publish(rmsg); });

    ntrip_client.set_report_interval(report_interval);
    ntrip_client.Run();
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Maybe take longer?

    ros::Rate loop_rate(10); // Adjust loop rate as needed
    while (ros::ok() && ntrip_client.service_is_running())
    {
        ros::spinOnce(); // Allow ROS to process messages and call registered callbacks
        loop_rate.sleep();
    }

    ntrip_client.Stop();
    return 0;
}
