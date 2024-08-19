
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

#include "xsens_time_handler.h"

XsensTimeHandler::XsensTimeHandler() : m_time_option(0), m_prevSampleTimeFine(0), isFirstFrame(true) {}

ros::Time XsensTimeHandler::convertUtcTimeToRosTime(const XsDataPacket &packet)
{
    std::lock_guard<std::mutex> lock(m_mutex);  // Lock the mutex at the beginning of the method
    /*
    if m_time_option=0 and containsUtc, set time to utc
    else if {m_time_option=0 or 1, {doesn't have utc, but has containsSampleTimeFine}}, set time to sampletimefine
    else set time to now
    */
    if (m_time_option == 0 && packet.containsUtcTime())
    {
        //ROS_INFO("Rosnode time_option is utc time from MTi");
        XsTimeInfo utcTime = packet.utcTime();
        struct tm timeinfo = {0};

        timeinfo.tm_year = utcTime.m_year - 1900;
        timeinfo.tm_mon = utcTime.m_month - 1;
        timeinfo.tm_mday = utcTime.m_day;
        timeinfo.tm_hour = utcTime.m_hour;
        timeinfo.tm_min = utcTime.m_minute;
        timeinfo.tm_sec = utcTime.m_second;

        time_t epochSeconds = timegm(&timeinfo);

        return ros::Time(epochSeconds, utcTime.m_nano);
    }
    else if ((m_time_option == 0 && !packet.containsUtcTime() &&  packet.containsSampleTimeFine() ) || (m_time_option == 1 && packet.containsSampleTimeFine()))
    {
        //ROS_INFO("Rosnode time_option is SampleTimeFine from MTi");
        uint32_t currentSampleTimeFine = packet.sampleTimeFine();

        if (isFirstFrame)
        {
            //ROS_INFO("time_option is mti_sampletime, first frame, timestamp: %u", currentSampleTimeFine);
            isFirstFrame = false;
            m_firstUTCTimestamp = ros::Time::now();
            m_prevSampleTimeFine = currentSampleTimeFine;
            return m_firstUTCTimestamp;
        }
        else
        {
            int64_t timeDiff = static_cast<int64_t>(currentSampleTimeFine) - static_cast<int64_t>(m_prevSampleTimeFine);

            // Checking for wraparound
            if (timeDiff < 0)
            {
                // Check if the difference is significant enough to be considered a wraparound
                if (timeDiff < -static_cast<int64_t>(m_RollOver / 2))
                {
                    // If wrap around occurred, adjust timeDiff
                    //ROS_INFO("time_option is mti_sampletime, Wraparound Detected. Current: %u, Previous: %u", currentSampleTimeFine, m_prevSampleTimeFine);
                    timeDiff += m_RollOver;
                }
                // else
                // {
                //     //// No adjustment on the cases when the packet comes later with a smaller sampleTimeFine.
                //     ROS_WARN("Minor timestamp decrement detected but not considered as wraparound. Current: %u, Previous: %u", currentSampleTimeFine, m_prevSampleTimeFine);
                // }
            }

            ros::Duration deltaTime(timeDiff * 0.0001); // Convert to seconds using the multiplier 0.0001
            m_firstUTCTimestamp += deltaTime;

            m_prevSampleTimeFine = currentSampleTimeFine;
            return m_firstUTCTimestamp;
        }
    }
    else
    {
        //ROS_INFO("Rosnode time_option is using host controller time");
        return ros::Time::now(); // returns ros time
    }
}

void XsensTimeHandler::setTimeOption(const int&option)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    m_time_option = option;
}

void XsensTimeHandler::setRollover(const uint32_t &rollOver)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    m_RollOver = rollOver;
}
