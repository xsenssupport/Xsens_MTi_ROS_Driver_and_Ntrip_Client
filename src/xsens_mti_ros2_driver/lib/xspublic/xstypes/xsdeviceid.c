
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

#include "xsdeviceid.h"
#include "xsstring.h"
#include "xsdid.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/*! \class XsDeviceId
	\brief Contains an Xsens device ID and provides operations for determining the type of device
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \returns The legacy bit used to identify legacy or new XsDeviceId format */
uint64_t XsDeviceId_legacyBit(const struct XsDeviceId* thisPtr)
{
	(void)thisPtr;
	return XS_DID64_BIT;
}

/*! \brief Test if the device ID represents a legacy device identification
	\returns true if this XsDeviceId represents a legacy device identification
*/
int XsDeviceId_isLegacyDeviceId(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID64_BIT) == 0);
}

/*! \brief Test if the device ID represents an Mti 1-series device
	\returns true if this XsDeviceId represents an Mti-1 series device
*/
int XsDeviceId_isMtiX(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X_MPU);
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && (deviceFamily < 10));
	}
}

/*! \brief Test if the device ID represents an Mti 10-series device
	\returns true if this XsDeviceId represents an Mti-10 series device
*/
int XsDeviceId_isMtiX0(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X0);
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && ((deviceFamily >= 10) && (deviceFamily < 100)));
	}
}

/*! \brief Test if the device ID represents an Mti 100-series device
	\returns true if this XsDeviceId represents an Mti-100 series device
*/
int XsDeviceId_isMtiX00(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X00);
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		if ((deviceFamily != 0) && (deviceFamily >= 100 && deviceFamily <= 300))
			return 1;
		else if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) == 0)
		{
			deviceFamily = atoi(&thisPtr->m_productCode[6]);
			if ((deviceFamily != 0) && (deviceFamily >= 100))
				return 1;
		}
	}
	return 0;
}

/*! \brief Test if the device ID represents an Mtig 700 device
	\returns true if this XsDeviceId represents an Mtig 700 series device
*/
int XsDeviceId_isMtigX00(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) &&
			((thisPtr->m_deviceId & ~XS_DID_TYPEL_COMM_MASK) < XS_DID_MK4TYPE_MT_710_RANGE_START);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[6]);
		return ((deviceFamily != 0) && (deviceFamily == 700));
	}
}

/*! \brief Test if the device ID represents an Mtig 710 device
	\returns true if this XsDeviceId represents an Mtig 710 series device
*/
int XsDeviceId_isMtigX10(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) &&
			((thisPtr->m_deviceId & ~XS_DID_TYPEL_COMM_MASK) >= XS_DID_MK4TYPE_MT_710_RANGE_START);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[6]);
		return ((deviceFamily != 0) && (deviceFamily == 710));
	}
}

/*! \brief Test if the device ID represents an MTi-600 series device
	\returns true if this XsDeviceId represents an Mti-600 series device
*/
int XsDeviceId_isMti6X0(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && ((deviceFamily > 600) && (deviceFamily < 700)));
	}
}

/*! \brief Test if the device ID represents an Avior device
	\returns true if this XsDeviceId represents an Avior device
*/
int XsDeviceId_isAvior(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else if (thisPtr->m_productCode[0] == 'A')
		return 1;

	return 0;
}

/*! \brief Test if the device ID represents an Sirius device
	\returns true if this XsDeviceId represents an Sirius device
*/
int XsDeviceId_isSirius(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else if (thisPtr->m_productCode[0] == 'S')
		return 1;

	return 0;
}

/*! \brief Test if the device ID represents an MTi-3X0 device
	\returns true if this XsDeviceId represents an MTi-3X0 series device
*/
int XsDeviceId_isMti3X0(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_3X0);
	else
		return 0;
}

/*! \brief Test if the device ID represents a Glove series device
	\returns true if this XsDeviceId represents a Glove series device
*/
int XsDeviceId_isGlove(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (thisPtr->m_deviceId == XS_DID_GLOVEMASTER_LEFT || thisPtr->m_deviceId == XS_DID_GLOVEMASTER_RIGHT)
			return 1;

		return 0;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "Glove", 5) != 0)
			return 0;

		return 1;
	}
}

/*! \brief Return the side the device should be worn on.
	\details Currently this only applies to the Xsens Glove product line. For other devices XHI_Unknown will be returned.
	\return The hand id of the provided Glove XsDeviceId, XHI_Unknown if the device ID does not represent a Glove
*/
XsHandId XsDeviceId_side(struct XsDeviceId const* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (thisPtr->m_deviceId == XS_DID_GLOVEMASTER_LEFT)
			return XHI_LeftHand;
		if (thisPtr->m_deviceId == XS_DID_GLOVEMASTER_RIGHT)
			return XHI_RightHand;
		return XHI_Unknown;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "Glove", 5) != 0)
			return XHI_Unknown;

		switch (thisPtr->m_productCode[8])
		{
			case 'L':
				return XHI_LeftHand;
			case 'R':
				return XHI_RightHand;
			default:
				return XHI_Unknown;
		}
	}
}

/*! \brief Test if the device ID represents a Dot series device
	\returns true if this XsDeviceId represents a Dot series device
*/
int XsDeviceId_isDot(struct XsDeviceId const* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else
	{
		if (memcmp(thisPtr->m_productCode, "XS-", 3) != 0)
			return 0;

		return 1;
	}
}

/*! \brief Test if this device ID represents an Rugged Version
	\returns true if this XsDeviceId represents an Rugged Sensor
*/
int XsDeviceId_isRugged(struct XsDeviceId const* thisPtr)
{
	if (XsDeviceId_isMti6X0(thisPtr))
	{
		if (thisPtr->m_productCode[7] == 'R' || thisPtr->m_productCode[7] == 'G')
			return 1;
	}
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[5] == 'A' || thisPtr->m_productCode[5] == 'B')
			return 1;
	}
	return 0;

}

/*! \brief Test if this device ID represents a device with internal GNSS receiver
	\returns true if this XsDeviceId represents a Sensor with internal GNSS
*/
int XsDeviceId_hasInternalGnss(struct XsDeviceId const* thisPtr)
{
	if (XsDeviceId_isMti6X0(thisPtr))
	{
		if (thisPtr->m_productCode[7] == 'G')
			return 1;
	}
	if (XsDeviceId_isMtigX10(thisPtr))
		return 1;
	return 0;

}

/*! \brief Test if this device ID represents an MTw
	\returns true if this XsDeviceId represents an MTw
*/
int XsDeviceId_isMtw(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtw2(thisPtr) || XsDeviceId_isMtw2Obskur(thisPtr);
}

/*! \brief Test if this device ID represents an MTw2.
	\returns true if this XsDeviceId represents an MTw2
*/
int XsDeviceId_isMtw2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_MTW2(thisPtr->m_deviceId);
	else
		return (memcmp(thisPtr->m_productCode, "MTw2", 4) == 0);
}

/*! \brief Test if this device ID represents an MTw2Obskur device.
	\returns true if this XsDeviceId represents an MTw2 for Obskur
*/
int XsDeviceId_isMtw2Obskur(const struct XsDeviceId* thisPtr)
{
	return XS_DID_MTW2OBSKUR(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an MTx
	\returns true if this XsDeviceId represents an MTx
*/
int XsDeviceId_isMtx(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtx2(thisPtr);
}

/*! \brief Test if this device ID represents an MTx2
	\returns true if this XsDeviceId represents an MTx2
*/
int XsDeviceId_isMtx2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_MTX2(thisPtr->m_deviceId);
	else
		return (memcmp(thisPtr->m_productCode, "MTx2", 4) == 0);
}

/*! \brief Test if this device ID represents an MTx3
	\returns true if this XsDeviceId represents an MTx3
*/
int XsDeviceId_isMtx3(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else
		return (memcmp(thisPtr->m_productCode, "XSL-MTX3", 8) == 0);
}

/*! \brief Test if this device ID represents a bodyhub device.
	\returns true if this XsDeviceId represents a bodyhub device
*/
int XsDeviceId_isBodyHub(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else
		return (memcmp(thisPtr->m_productCode, "XSL-HUB", 7) == 0);
}

/*! \brief Test if this device ID represents a bodypack (any version) device.
	\returns true if this XsDeviceId represents a bodypack (any version) device
*/
int XsDeviceId_isBodyPack(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_BODYPACK(thisPtr->m_deviceId) || (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER);
	else
		return ((memcmp(thisPtr->m_productCode, "BodyPack", 8) == 0) || (memcmp(thisPtr->m_productCode, "BPACK", 5) == 0));
}

/*! \brief Test if this device ID represents a bodypack V1 device.
	\returns true if this XsDeviceId represents a bodypack V1 device
*/
int XsDeviceId_isBodyPackV1(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_BODYPACK(thisPtr->m_deviceId) || (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER);
	else
		return 0;	// all BPv1 devices use legacy device IDs
}

/*! \brief Test if this device ID represents a bodypack V2 device.
	\returns true if this XsDeviceId represents a bodypack V2 device
*/
int XsDeviceId_isBodyPackV2(const struct XsDeviceId* thisPtr)
{
	if (!XsDeviceId_isLegacyDeviceId(thisPtr))
		return memcmp(thisPtr->m_productCode, "BPACK-V2", 8) == 0;
	else
		return 0;	// all BPv2 devices use big device IDs
}

/*! \brief Test if this device ID represents a Wireless Master device (Awinda Station, Awinda Dongle, Awinda OEM)
	\returns true if this XsDeviceId represents a Wireless Master device
*/
int XsDeviceId_isWirelessMaster(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER) &&
			!XsDeviceId_isBodyPack(thisPtr) &&
			!XsDeviceId_isSyncStationX(thisPtr);
	}
	else
		return (memcmp(thisPtr->m_productCode, "AW-", 3) == 0);
}

/*! \brief Test if this device ID represents an Awinda device.
	\returns true if this XsDeviceId represents an Awinda device
*/
int XsDeviceId_isAwindaX(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda Station.
	\returns true if this XsDeviceId represents an Awinds Station
*/
int XsDeviceId_isAwindaXStation(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Station(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda Dongle.
	\returns true if this XsDeviceId represents an Awinda Dongle
*/
int XsDeviceId_isAwindaXDongle(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Dongle(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda OEM board.
	\returns true if this XsDeviceId represents an Awinda OEM board
*/
int XsDeviceId_isAwindaXOem(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Oem(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda2 device.
	\returns true if this XsDeviceId represents an Awinda2 device
*/
int XsDeviceId_isAwinda2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_AWINDA2(thisPtr->m_deviceId);
	else
		return (memcmp(thisPtr->m_productCode, "AW-", 3) == 0);
}

/*! \brief Test if this device ID represents an Awinda2 Station.
	\returns true if this XsDeviceId represents an Awinda2 Station
*/
int XsDeviceId_isAwinda2Station(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_AWINDA2_STATION(thisPtr->m_deviceId);
	else
		return memcmp(thisPtr->m_productCode, "AW-A2", 5) == 0;
}

/*! \brief Test if this device ID represents an Awinda2 Dongle.
	\returns true if this XsDeviceId represents an Awinda2 Dongle
*/
int XsDeviceId_isAwinda2Dongle(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return XS_DID_AWINDA2_DONGLE(thisPtr->m_deviceId);
	else
		return memcmp(thisPtr->m_productCode, "AW-DNG2", 7) == 0;
}

/*! \brief Test if this device ID represents an Awinda2 Dongle with antenna.
	\returns true if this XsDeviceId represents an Awinda2 Dongle with antenna
*/
int XsDeviceId_isAwinda2DongleAntenna(const struct XsDeviceId* thisPtr)
{
	return memcmp(thisPtr->m_productCode, "AW-DNG2-ANT", 11) == 0;
}

/*! \brief Test if this device ID represents an Awinda2 OEM board.
	\returns true if this XsDeviceId represents an Awinda2 OEM board
*/
int XsDeviceId_isAwinda2Oem(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2_OEM(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation.
	\returns true if this XsDeviceId represents a SyncStation
*/
int XsDeviceId_isSyncStationX(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation v2.
	\returns true if this XsDeviceId represents a SyncStation v2
*/
int XsDeviceId_isSyncStation2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a Hardware In the Loop test device.
	\returns true if this XsDeviceId represents a Hardware In the Loop test device
*/
int XsDeviceId_isHilDevice(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return 0;
	else
		return strcmp(thisPtr->m_productCode, "HILDEVICE") == 0;
}

/*! \brief Test if this device ID represents an IMU.
	\returns true if this XsDeviceId represents an IMU
*/
int XsDeviceId_isImu(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_MPU) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_10) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_100) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_310));
	}
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[2] == 'M')
			return 1;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6 || deviceFamily == 8 || deviceFamily == 9)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 1);
	}
	return 0;
}

/*! \brief Test if this device ID represents a VRU.
	\returns true if this XsDeviceId represents a VRU
*/
int XsDeviceId_isVru(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_MPU) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_20) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_200) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_320));
	}
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[2] == 'V')
			return 1;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6 || deviceFamily == 8 || deviceFamily == 9)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 2);
	}
	return 0;
}

/*! \brief Test if this device ID represents an AHRS.
	\returns true if this XsDeviceId represents an AHRS
*/
int XsDeviceId_isAhrs(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_MPU) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_30) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_300) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_330));
	}
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[2] == 'A')
			return 1;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6 || deviceFamily == 8 || deviceFamily == 9)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 3);
	}
	return 0;
}

/*! \brief Test if this device ID represents an GNSS (capable) device.
	\returns true if this XsDeviceId represents a GNSS capable device
*/
int XsDeviceId_isGnss(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) ||
				((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_7_MPU)
				||(thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_8_MPU);
	}
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[2] == 'G')
			return 1;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		if (deviceFamily == 7)
			return 1;
		else if ((deviceFamily == 670) || (deviceFamily == 680) || (deviceFamily == 870) || (deviceFamily == 880))
			return 1;
		else
		{
			if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
				return 0;

			deviceFamily = atoi(&thisPtr->m_productCode[6]);
			return (deviceFamily == 700 || deviceFamily == 710);
		}
	}
	return 0;
}

/*! \brief Test if this device ID represents an RTK (capable) device.
	\returns true if this XsDeviceId represents a RTK capable device
*/
int XsDeviceId_isRtk(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_8_MPU);
	else if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
	{
		if (thisPtr->m_productCode[2] == 'R')
			return 1;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily == 680) || (deviceFamily == 880));
	}
	return 0;
}

/*! \brief Test if this device ID represents any of the container devices such as Bodypack and Awinda Station
	\returns true if this XsDeviceId represents a container device
*/
int XsDeviceId_isContainerDevice(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isBodyPack(thisPtr) || XsDeviceId_isWirelessMaster(thisPtr);
}

/*! \brief Test if this device ID represents an MT device (any Mti, Mtig, Mtx or Mtw)
	\returns true if this XsDeviceId represents an MT device
*/
int XsDeviceId_isMt(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMti(thisPtr) || XsDeviceId_isMtig(thisPtr) || XsDeviceId_isMtw(thisPtr) || XsDeviceId_isMtx(thisPtr));
}

/*! \brief Test if this device ID represents an MTi device (1, 10 or 100 series, 1 includes MTi-7)
	\returns true if this XsDeviceId represents an MTi device
*/
int XsDeviceId_isMti(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMtiX(thisPtr) ||
			XsDeviceId_isMtiX0(thisPtr) ||
			XsDeviceId_isMtiX00(thisPtr) ||
			XsDeviceId_isMti3X0(thisPtr) ||
			XsDeviceId_isMti6X0(thisPtr) ||
			XsDeviceId_isAvior(thisPtr) ||
			XsDeviceId_isSirius(thisPtr));
}

/*! \brief Test if this device ID represents an MTig device (700 or 710 series)
	\returns true if this XsDeviceId represents an MTig device
*/
int XsDeviceId_isMtig(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMtigX00(thisPtr) || XsDeviceId_isMtigX10(thisPtr));
}

/*! \brief Test if this device ID represents an Mk4 generation MT device
	\returns true if this XsDeviceId represents an Mk4 generation device
*/
int XsDeviceId_isMtMark4(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && ((thisPtr->m_deviceId & XS_DID_TYPEL_MK5) != XS_DID_TYPEL_MK5);
	else
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && thisPtr->m_hardwareVersion < 0x300;
}

/*! \brief Test if this device ID represents an Mk5 generation MT device
	\returns true if this XsDeviceId represents an Mk5 generation device
*/
int XsDeviceId_isMtMark5(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && ((thisPtr->m_deviceId & XS_DID_TYPEL_MK5) == XS_DID_TYPEL_MK5);
	else
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && thisPtr->m_hardwareVersion >= 0x300;
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

/*! \brief Get a string with a readable representation of this device ID.
	\param str The string to write to
*/
void XsDeviceId_toString(const XsDeviceId* thisPtr, XsString* str)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		char device[9];
		sprintf(device, "%08" PRINTF_INT32_MODIFIER "X", (uint32_t)thisPtr->m_deviceId);
		XsString_assign(str, 8, device);
	}
	else
	{
		XsSize n;
		char device[23];
		if (thisPtr->m_subDevice)
			n = (XsSize)(ptrdiff_t) sprintf(device, "%010" PRINTF_INT64_MODIFIER "X/%hu", thisPtr->m_deviceId, (unsigned short int) thisPtr->m_subDevice);
		else
			n = (XsSize)(ptrdiff_t) sprintf(device, "%010" PRINTF_INT64_MODIFIER "X", thisPtr->m_deviceId);
		XsString_assign(str, n, device);
	}
}

/*! \brief Read a device ID from the supplied string.
	\param str The string to interpret
*/
void XsDeviceId_fromString(XsDeviceId* thisPtr, const XsString* str)
{
	uint64_t tmp = 0;
	uint16_t sub = 0;
	if (!thisPtr || !str || !str->m_data)
		return;

	if (isalpha(str->m_data[0]))
		return;

	int result = sscanf(str->m_data, "%" PRINTF_INT64_MODIFIER "x/%hu", &tmp, &sub);
	if (result >= 1)
		thisPtr->m_deviceId = tmp;
	if (result >= 2)
		thisPtr->m_subDevice = sub;
	else
		thisPtr->m_subDevice = 0;
}

/*! \brief Get a string with a readable representation of this device ID. Either full or as a type
	\param makeType Boolean whether the deviceid is changed to a type instead of the full deviceid
	\param str The string to write to
*/
void XsDeviceId_toDeviceTypeString(const XsDeviceId* thisPtr, XsString* str, int makeType)
{
	XsDeviceId deviceType = XSDEVICEID_INITIALIZER;
	if (makeType)
		XsDeviceId_deviceType(thisPtr, 1, &deviceType);
	else
		deviceType = *thisPtr;
	char device[50];
	XsSize n;
	if (thisPtr->m_hardwareVersion == 0)
		n = (XsSize)(ptrdiff_t) sprintf(device, "%s%s%08X.%08X", deviceType.m_productCode, deviceType.m_productCode[0] ? "_" : "", (uint32_t)deviceType.m_deviceId, thisPtr->m_productVariant);
	else
		n = (XsSize)(ptrdiff_t) sprintf(device, "%s%s%08X.%08X.%d_%d", deviceType.m_productCode, deviceType.m_productCode[0] ? "_" : "", (uint32_t)deviceType.m_deviceId, thisPtr->m_productVariant, (uint8_t)((thisPtr->m_hardwareVersion & 0xFF00) >> 8), (uint8_t)(thisPtr->m_hardwareVersion & 0xFF));
	XsString_assign(str, n, device);
}

/*! \brief Read a device ID from the supplied string.
	\param str The string to interpret
*/
void XsDeviceId_fromDeviceTypeString(XsDeviceId* thisPtr, const XsString* str)
{
	uint32_t id = 0;
	int hwRevH = 0, hwRevL = 0;
	uint32_t variant;
	char productCode[24];
	if (!thisPtr || !str || !str->m_data)
		return;
	int result = sscanf(str->m_data, "%24[^_]_%08X.%08X.%d_%d", productCode, &id, &variant, &hwRevH, &hwRevL);
	if (result == 5 || result == 3)
	{
		thisPtr->m_deviceId = id;
		thisPtr->m_hardwareVersion = (uint16_t)((uint16_t)hwRevH << 8) + (uint16_t)hwRevL;
		thisPtr->m_productVariant = variant;
		strcpy(thisPtr->m_productCode, productCode);
	}
}

/*!	\brief Test if the device ID is a valid id (not 0).
	\returns true if the device ID is vallid
*/
int XsDeviceId_isValid(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return thisPtr->m_deviceId != 0;
	else
		return thisPtr->m_productCode[0] != 0 && thisPtr->m_deviceId != 0;
}

/*! \brief Swap the contents of \a a with those of \a b
	\param a Pointer to first deviceId
	\param b Pointer to second deviceId
*/
void XsDeviceId_swap(XsDeviceId* a, XsDeviceId* b)
{
	XsDeviceId tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \brief Returns true if this is equal to \a other or this is a type-specifier that matches \a other
	\param other The deviceid to compare this deviceid to
	\return True if the deviceids are equal or the type-specifier matches
*/
int XsDeviceId_contains(XsDeviceId const* thisPtr, XsDeviceId const* other)
{
	if (thisPtr == other)
		return 1;
	if (thisPtr->m_deviceId == other->m_deviceId)
		return 1;
	if (thisPtr->m_deviceId & XS_DID_ID_MASK) // NOTE: This produces incorrect results for device ids ending with 0000. See MVN-3876
		return 0;
	XsDeviceId typeThis, typeOther;
	XsDeviceId_deviceType(thisPtr, 1, &typeThis);
	XsDeviceId_deviceType(other, 1, &typeOther);
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return typeThis.m_deviceId == typeOther.m_deviceId;
	else
	{
		if (strcmp(typeThis.m_productCode, typeOther.m_productCode) == 0)
			return 1;
	}

	return 0;
}

/*! \brief Returns true if the ID is just a device type, not an actual device ID
	\returns true if the ID is just a device type
*/
int XsDeviceId_isType(XsDeviceId const* thisPtr)
{
	// true when we have a valid type (not a broadcast) and a 0 ID
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return	!(thisPtr->m_deviceId & XS_DID_ID_MASK);
	else
		return	!(thisPtr->m_deviceId & ~XS_DID64_BIT);
}

/*! \brief Returns the name of the type of device identified by this id
	\param str String to write to
*/
void XsDeviceId_typeName(XsDeviceId const* thisPtr, XsString* str)
{
	if (!str)
		return;

	if (!thisPtr)
	{
		XsString_assignCharArray(str, "invalid");
		return;
	}

	if (XsDeviceId_isAwinda2Station(thisPtr))
		XsString_assignCharArray(str, "Awinda Station v2");
	else if (XsDeviceId_isAwinda2Dongle(thisPtr))
		XsString_assignCharArray(str, "Awinda Dongle v2");
	else if (XsDeviceId_isAwinda2Oem(thisPtr))
		XsString_assignCharArray(str, "Awinda OEM v2");
	else if (XsDeviceId_isMtw2(thisPtr))
		XsString_assignCharArray(str, "MTw2");
	else if (XsDeviceId_isMtx2(thisPtr))
		XsString_assignCharArray(str, "MTx2");
	else if (XsDeviceId_isBodyPackV2(thisPtr))
		XsString_assignCharArray(str, "BPACK-V2");
	else if (XsDeviceId_isBodyPack(thisPtr))
		XsString_assignCharArray(str, "Bodypack");
	else if (XsDeviceId_isSyncStation2(thisPtr))
		XsString_assignCharArray(str, "Sync Station v2");
	else if (XsDeviceId_isMti6X0(thisPtr))
	{
		XsSize length = 7;
		if (thisPtr->m_productCode[7] != '-')
			length = 8;
		XsString_assign(str, length, thisPtr->m_productCode);
	}
	else if (XsDeviceId_isMti3X0(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "MTi-310");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "MTi-320");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "MTi-330");
	}
	else if (XsDeviceId_isMtiX(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "MTi-1");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "MTi-2");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "MTi-3");
		else if (XsDeviceId_isRtk(thisPtr))
			XsString_assignCharArray(str, "MTi-8");
		else if (XsDeviceId_isGnss(thisPtr))
			XsString_assignCharArray(str, "MTi-7");
	}
	else if (XsDeviceId_isMtiX0(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "MTi-10");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "MTi-20");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "MTi-30");
	}
	else if (XsDeviceId_isMtiX00(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "MTi-100");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "MTi-200");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "MTi-300");
		else if (XsDeviceId_isMtigX10(thisPtr))
			XsString_assignCharArray(str, "MTi-G-710");
		else if (XsDeviceId_isMtigX00(thisPtr))
			XsString_assignCharArray(str, "MTi-G-700");
	}
	else if (XsDeviceId_isAvior(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "Xsens Avior IMU");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "Xsens Avior VRU");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "Xsens Avior AHRS");
	}
	else if (XsDeviceId_isSirius(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "Xsens Sirius IMU");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "Xsens Sirius VRU");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "Xsens Sirius AHRS");
		else if (XsDeviceId_isMtigX10(thisPtr))
			XsString_assignCharArray(str, "Xsens Sirius GNSS/INS");
		else if (XsDeviceId_isMtigX00(thisPtr))
			XsString_assignCharArray(str, "Xsens Sirius RTK GNSS/INS");
	}
	else if (XsDeviceId_isGlove(thisPtr))
		XsString_assignCharArray(str, "Glove");
	else if (XsDeviceId_isMtw2Obskur(thisPtr))
		XsString_assignCharArray(str, "MTw2 Obskur");
	else
		XsString_assignCharArray(str, "Unknown");
}

/*! \brief Returns the type of device identified by this id
	\param type Type to write to
*/
void XsDeviceId_type(struct XsDeviceId const* thisPtr, struct XsDeviceId* type)
{
	if (!type)
		return;
	if (!XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		type->m_deviceId = XS_DID64_BIT;
		XsString typeName = XsString_INITIALIZER;
		XsDeviceId_typeName(thisPtr, &typeName);
		strcpy(type->m_productCode, typeName.m_data);
	}
	else
		type->m_deviceId = thisPtr->m_deviceId & XS_DID_FULLTYPE_MASK;
}

/*! \brief Returns the device type identified by this id (eg 10, 300 and Awinda2 Master)
	\param detailed Return detailed type information
	\param type Type to write to
*/
void XsDeviceId_deviceType(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type)
{
	if (!type)
		return;
	if (!XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (XsDeviceId_isSirius(thisPtr) || XsDeviceId_isAvior(thisPtr))
		{
			type->m_deviceId = XS_DID64_BIT;
			strncpy(type->m_productCode, thisPtr->m_productCode, 3);
			type->m_productCode[3] = 0;
			if (detailed)
			{
				type->m_hardwareVersion = thisPtr->m_hardwareVersion;
				type->m_productVariant = thisPtr->m_productVariant;
			}
			else
			{
				type->m_hardwareVersion = 0;
				type->m_productVariant = 0;
			}
		}
		else if (XsDeviceId_isMti6X0(thisPtr))
		{
			type->m_deviceId = XS_DID64_BIT;
			strncpy(type->m_productCode, thisPtr->m_productCode, 7);
			type->m_productCode[7] = 0;
			if (detailed)
			{
				type->m_hardwareVersion = thisPtr->m_hardwareVersion;
				type->m_productVariant = thisPtr->m_productVariant;
			}
			else
			{
				type->m_hardwareVersion = 0;
				type->m_productVariant = 0;
			}
		}
		else
		{
			type->m_deviceId = XS_DID64_BIT;
			strncpy(type->m_productCode, thisPtr->m_productCode, 24);
			type->m_productCode[23] = 0;
			if (detailed)
			{
				type->m_hardwareVersion = thisPtr->m_hardwareVersion;
				type->m_productVariant = thisPtr->m_productVariant;
			}
			else
			{
				type->m_hardwareVersion = 0;
				type->m_productVariant = 0;
			}
		}
	}
	else
	{
		XsDeviceId_deviceTypeMask(thisPtr, detailed, type);
		type->m_deviceId = thisPtr->m_deviceId & type->m_deviceId;
		strcpy(type->m_productCode, thisPtr->m_productCode);
	}
}

/*! \brief Returns the mask which can be used to get the detailed device type (eg 10, 300 and Awinda2 Master)
	\param detailed Return detailed type information
	\param type Type to write to
*/
void XsDeviceId_deviceTypeMask(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (XsDeviceId_isMti3X0(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? XS_DID_GP_MASK : 0));
		else if (XsDeviceId_isMtiX(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GP_MASK | XS_DID_TYPEL_MASK) : 0));
		else if ((XsDeviceId_isMtiX0(thisPtr)) || (XsDeviceId_isMtiX00(thisPtr)))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GP_MASK | XS_DID_TYPEL_MK5) : 0));
		else if (XsDeviceId_isAwindaX(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? XS_DID_GP_MASK : 0));
		else if (XsDeviceId_isSyncStationX(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? XS_DID_GP_MASK : 0));
		else if (XsDeviceId_isMtw(thisPtr) || XsDeviceId_isMtx(thisPtr))
			type->m_deviceId = (XS_DID_TYPE_MASK | (detailed ? XS_DID_GP_MASK : 0));
		else if (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER)
			type->m_deviceId = XS_DID_ABMCLOCKMASTER;
		else
			type->m_deviceId = XS_DID_TYPEH_MASK;
	}
	else
		type->m_deviceId = XS_DID64_BIT;
}

/*! \brief Returns the base part (lower 4 bytes) of the device Id
*/
uint16_t XsDeviceId_basePart(struct XsDeviceId const* thisPtr)
{
	return (uint16_t)(thisPtr->m_deviceId & XS_DID_BASE_ID_MASK);
}

/*! @} */
