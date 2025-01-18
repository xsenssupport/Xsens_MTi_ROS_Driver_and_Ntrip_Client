
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

#ifndef XSSENSORRANGES_H
#define XSSENSORRANGES_H

#include "xstypesconfig.h"
#include "xsstring.h"
#include "xsdeviceid.h"

#ifdef __cplusplus
extern "C" {
#endif

enum HardwareManufacturerType
{
	HMT_MT	= 0,
	HMT_None
};
typedef enum HardwareManufacturerType HardwareManufacturerType;

XSTYPES_DLL_API void findHardwareTypeC(const XsDeviceId* deviceId, XsString* resultValue);
XSTYPES_DLL_API HardwareManufacturerType findHardwareManufacturerC(const XsDeviceId* deviceId);

XSTYPES_DLL_API double accelerometerRangeC(const XsDeviceId* deviceId);
XSTYPES_DLL_API double gyroscopeRangeC(const XsDeviceId* deviceId);

XSTYPES_DLL_API double actualAccelerometerRangeC(const XsDeviceId* deviceId);
XSTYPES_DLL_API double actualGyroscopeRangeC(const XsDeviceId* deviceId);

#ifdef __cplusplus
}

inline static XsString findHardwareType(const XsDeviceId& deviceId)
{
	XsString rv;
	findHardwareTypeC(&deviceId, &rv);
	return rv;
}
inline static HardwareManufacturerType findHardwareManufacturer(const XsDeviceId& deviceId)
{
	return findHardwareManufacturerC(&deviceId);
}
inline static double accelerometerRange(const XsDeviceId& deviceId)
{
	return accelerometerRangeC(&deviceId);
}
inline static double gyroscopeRange(const XsDeviceId& deviceId)
{
	return gyroscopeRangeC(&deviceId);
}
inline static double actualAccelerometerRange(const XsDeviceId& deviceId)
{
	return actualAccelerometerRangeC(&deviceId);
}
inline static double actualGyroscopeRange(const XsDeviceId& deviceId)
{
	return actualGyroscopeRangeC(&deviceId);
}
#endif

#endif
