
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

#include "xssensorranges.h"
#include <cassert>

extern "C" {

	/*! \brief Return the hardware manufacturer from \a productCode
	*/
	HardwareManufacturerType findHardwareManufacturerC(const XsDeviceId* deviceId)
	{
		if (deviceId->isMt())
			return HMT_MT;

		return HMT_None;
	}

	/*! \brief Return the hardware type from \a productCode
	*/
	void findHardwareTypeC(const XsDeviceId* deviceId, XsString* resultValue)
	{
		if (!resultValue)
			return;

		// Clear upfront, so we can exit when needed
		resultValue->clear();

		if (findHardwareManufacturer(*deviceId) != HMT_MT)
			return;

		if (XsDeviceId_isSirius(deviceId) || XsDeviceId_isAvior(deviceId))
		{
			// Return the value Sirius / Avior should have
			// This way rest of the code does not need exceptions
			*resultValue = "A4G3";
		}
		else if (XsDeviceId_isMti6X0(deviceId))
		{
			// Return the value Mti6X0 should have
			// This way rest of the code does not need exceptions
			*resultValue = "A1G6";
		}
		else
		{
			// Return the string starting from the first range field.
			// e.g. A8G4 for the MTi-30-2A8G4 product code
			// e.g. A70G20 for the MTw-38A70G20 product code
			// Also check that both A and G are present within 3 bytes as they should be.
			const char* A = strchr(deviceId->productCodeData(), 'A');
			if (!A)
				return;
			const char* G = strchr(A, 'G');
			if (!G)
				return;
			assert(G - A <= 3);
			*resultValue = A;
		}
	}

	/* Return the accelerometer range field */
	static char accelerometerRangeField(const XsDeviceId* deviceId)
	{
		XsString hwi;
		findHardwareTypeC(deviceId, &hwi);
		if (hwi.empty())
			return 0;
		return hwi.at(1);
	}

	/* Return the gyroscope range field */
	static char gyroscopeRangeField(const XsDeviceId* deviceId)
	{
		XsString hwi;
		findHardwareTypeC(deviceId, &hwi);
		if (hwi.empty())
			return 0;
		ptrdiff_t offset = hwi.find('G');
		if (offset < 0)
			return 0;
		return hwi.at(offset + 1);
	}

	/*! \brief The accelerometer range from product code \a productCode
	*/
	double accelerometerRangeC(const XsDeviceId* deviceId)
	{
		switch (findHardwareManufacturerC(deviceId))
		{
			case HardwareManufacturerType::HMT_MT:
				switch (accelerometerRangeField(deviceId))
				{
					case '1':
						return   100.0;
					case '2':
						return    20.0;
					case '3':
						return    17.0;
					case '4':
						return	  80.0;
					case '5':
						return    50.0;
					case '6':
						return    60.0;
					case '7':
						return   160.0;
					case '8':
					{
						uint8_t major = (deviceId->hardwareVersion() & 0xFF00) >> 8;
						if (major < 3)
							return 180.0;
						else
							return 200.0;
					}
					default:
						return 10000.0;
				}

			default:
				return 10000.0;
		}
	}

	/*! \brief The actual accelerometer range from product code \a productCode

		This is a measured value and possibly larger than what accelerometerRange() returns.
	*/
	double actualAccelerometerRangeC(const XsDeviceId* deviceId)
	{
		switch (findHardwareManufacturerC(deviceId))
		{
			case HardwareManufacturerType::HMT_MT:
				switch (accelerometerRangeField(deviceId))
				{
					case '1':
						return   100.0;
					case '2':
						return    20.0;
					case '3':
						return    17.0;
					case '5':
						return    50.0;
					case '6':
						return    60.0;
					case '7':
						return   160.0;
					case '8':
					{
						uint8_t major = (deviceId->hardwareVersion() & 0xFF00) >> 8;
						if (major < 3)
							return 180.0;
						else
							return 200.0;
					}
					default:
						return 10000.0;
				}

			default:
				return 10000.0;
		}
	}

	/*! \brief The gyroscope range from product code \a productCode
	*/
	double gyroscopeRangeC(const XsDeviceId* deviceId)
	{
		switch (findHardwareManufacturerC(deviceId))
		{
			case HardwareManufacturerType::HMT_MT:
				switch (gyroscopeRangeField(deviceId))
				{
					case '0':
						return  1000.0;
					case '1':
						return   150.0;
					case '2':
						return  1200.0;
					case '3':
						return   300.0;
					case '4':
						return   450.0;
					case '5':
						return  2500.0;
					case '6':
						return  1800.0;
					case '7':
						return   700.0;
					case '9':
						return   900.0;
					default:
						return 10000.0;
				}

			default:
				return 10000.0;
		}
	}

	/*! \brief The actual gyroscope range from product code \a productCode

		This is a measured value and possibly larger than what gyroscopeRange() returns.
	*/
	double actualGyroscopeRangeC(const XsDeviceId* deviceId)
	{
		switch (findHardwareManufacturerC(deviceId))
		{
			case HardwareManufacturerType::HMT_MT:
				switch (gyroscopeRangeField(deviceId))
				{
					case '0':
						return  1000.0;
					case '1':
						return   180.0;
					case '2':
						return  1700.0;
					case '3':
						return   420.0;
					case '4':
						return   450.0;
					case '5':
						return  2500.0;
					case '6':
						return  2000.0;
					case '7':
						return   700.0;
					case '9':
						return  1080.0;
					default:
						return 10000.0;
				}

			default:
				return 10000.0;
		}
	}

}
