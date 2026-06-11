
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef TRIGGERINTIMEPUBLISHER_H
#define TRIGGERINTIMEPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/TimeReference.h>
#include <xstypes/xsdataidentifier.h>
#include <xstypes/xstriggerindicationdata.h>

struct TriggerInTimePublisher : public PacketCallback
{
    ros::Publisher pub;

    TriggerInTimePublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<sensor_msgs::TimeReference>("imu/triggerin_time", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        // A hardware trigger on a SyncIn line arrives in its own MTData2 packet as a
        // trigger indication. MTi-600/Sirius/Avior expose two SyncIn lines, each with its
        // own data identifier; a single packet may carry both, so check each line and
        // publish one TimeReference per indication present.
        static const XsDataIdentifier triggerIds[] = { XDI_TriggerIn1, XDI_TriggerIn2 };

        for (XsDataIdentifier triggerId : triggerIds)
        {
            if (packet.containsTriggerIndication(triggerId))
            {
                // The trigger indication timestamp is in microseconds (1 MHz),
                // unlike SampleTimeFine which is in 0.1 ms units (10 kHz).
                const uint32_t TRIGGER_TIME_HZ = 1000000UL;
                const uint32_t ONE_GHZ = 1000000000UL;

                XsTriggerIndicationData trigger = packet.triggerIndication(triggerId);
                uint32_t t_trigger = trigger.m_timestamp;
                uint32_t sec = t_trigger / TRIGGER_TIME_HZ;
                uint32_t nsec = (t_trigger % TRIGGER_TIME_HZ) * (ONE_GHZ / TRIGGER_TIME_HZ);

                ros::Time trigger_time(sec, nsec);

                sensor_msgs::TimeReference msg;
                msg.header.stamp = timestamp;
                // msg.header.frame_id = unused
                msg.time_ref = trigger_time;
                msg.source = (triggerId == XDI_TriggerIn1) ? "TriggerIn1" : "TriggerIn2";

                pub.publish(msg);
            }
        }
    }
};

#endif
