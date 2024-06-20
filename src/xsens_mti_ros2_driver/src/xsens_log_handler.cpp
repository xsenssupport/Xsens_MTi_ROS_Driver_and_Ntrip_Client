
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

#include "xsens_log_handler.h"
#include <sys/stat.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cstdlib> // for getenv()

XsensLogHandler::XsensLogHandler()
{
    const char *home = getenv("HOME");
    if (home)
    {
        logDirectory = std::string(home) + "/xsens_log";
    }
    else
    {
        std::cerr << "Warning: Cannot get HOME environment variable. Using /tmp for log directory." << std::endl;
        logDirectory = "/tmp/xsens_log"; // Use /tmp as a safer fallback
    }
}

XsensLogHandler::~XsensLogHandler() {}

void XsensLogHandler::prepareLogDirectory()
{
    struct stat st;
    if (stat(logDirectory.c_str(), &st) == -1)
    {
        mkdir(logDirectory.c_str(), 0700); // Permissions set to owner read/write/execute
    }
}

std::string XsensLogHandler::getLogFileName() const
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    struct tm *tm_info = std::localtime(&in_time_t);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d_%H%M%S.mtb", tm_info);
    ss << logDirectory << "/" << buffer;

    return ss.str();
}
