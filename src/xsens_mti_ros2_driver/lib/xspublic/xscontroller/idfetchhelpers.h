
//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
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

#ifndef IDFETCHHELPERS_H
#define IDFETCHHELPERS_H

#include <xstypes/xsdeviceid.h>
#include <algorithm>

/* Helper functions for fetching of device/vendor/product ids from text strings */

/* Constant parts for string searching */
static const char gDevicePathVidString[] = "VID_";
static const char gDevicePathPidString[] = "PID_";

/* Is a character a valid hex character? */
static inline bool ishex(char foo)
{
	return (foo >= '0' && foo <= '9') || (foo >= 'a' && foo <= 'f') || (foo >= 'A' && foo <= 'F');
}

/* Fetch the device ID from the given device path */
static XsDeviceId deviceIdFromDevPath(std::string const& devpath)
{
	XsDeviceId deviceId;
	size_t last = devpath.size();
	while (last > 0 && ishex(devpath[last - 1]))
		--last;
	if (last != devpath.size())
		deviceId.fromString(XsString(devpath.substr(last)));
	return deviceId;
}

static uint16_t extractUint16(std::string const& string, size_t pos)
{
	if (pos >= string.size())
		return 0;

	const char* data = string.c_str() + pos;
	char* endptr = nullptr;
	auto result = std::strtoul(data, &endptr, 16);

	// We require exactly 4 valid characters
	return (endptr == data + 4) ? static_cast<uint16_t>(result) : 0;
}

static bool findVidPidPositionInString(std::string const& string, size_t & posVid, size_t & posPid)
{
	std::string normalizedString = string;
	std::transform(normalizedString.begin(), normalizedString.end(), normalizedString.begin(), ::toupper);
	posVid = normalizedString.find(gDevicePathVidString);
	posPid = normalizedString.find(gDevicePathPidString);
	if (posVid != std::string::npos && posPid != std::string::npos && posVid < posPid)
	{
		posVid += sizeof(gDevicePathVidString) - 1;
		posPid += sizeof(gDevicePathPidString) - 1;
		return true;
	}
	else
	{
		return false;
	}
}

/* Fetch the vendor ID from the given string */
static inline uint16_t vidFromString(std::string const& string)
{
	uint16_t vid = 0;
	size_t posVid;
	size_t posPid;
	if (findVidPidPositionInString(string, posVid, posPid))
		vid = extractUint16(string, posVid);
	return vid;
}

/* Fetch the product ID from the given string */
static inline uint16_t pidFromString(std::string const& string)
{
	uint16_t pid = 0;
	size_t posVid;
	size_t posPid;
	if (findVidPidPositionInString(string, posVid, posPid))
		pid = extractUint16(string, posPid);
	return pid;
}

#endif
