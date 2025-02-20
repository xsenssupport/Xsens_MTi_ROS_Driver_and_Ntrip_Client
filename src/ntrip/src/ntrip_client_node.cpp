#include "ntrip_client/ntrip_client.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ntrip_client");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ntrip_client::NtripClient client(nh, private_nh);

    if (!client.Start())
    {
        ROS_ERROR("Failed to start NTRIP client");
        return 1;
    }

    ros::spin();

    client.Stop();
    return 0;
}