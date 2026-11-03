
//  Copyright (c) 2003-2026 Xsens Technologies B.V. or subsidiaries worldwide.
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

#ifndef XSRAWGNSSHEADING_H
#define XSRAWGNSSHEADING_H

#include "pstdint.h"
#include "xstypedefs.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#elif defined (__ICCARM__)
_Pragma("pack(push, 1)")
#endif

#define XSRAWGNSSHEADING_INITIALIZER {0, 0,0,0,0,0,  0,0,0,0,0, 0}
#ifdef __cplusplus
#include <cstring>
#endif

/*!	\addtogroup enums Global enumerations
	@{
*/

/*! \enum XsGnssHeadingFlags
 *  \brief Defines the flags for the XsRawGnssHeading::m_flags field
*/
enum XsGnssHeadingFlags
{
	XGHF_GnssFixOk				= 0x00000001, //!< Set if there is a valid GNSS fix
	XGHF_DiffCorrectionsApplied = 0x00000002, //!< Set if differential corrections were applied
	XGHF_RelativePositionValid	= 0x00000004, //!< Set if the relative position, accuracy and baseline are valid
	XGHF_CarrSolnMask			= 0x00000018, //!< Mask for the 2 bit carrier phase range solution field, see XsGnssHeadingCarrierSolution
	XGHF_RelativeHeadingValid	= 0x00000040, //!< Set if the relative heading value is valid
};

/*! \enum XsGnssHeadingFlagOffset
 *  \brief Defines the bit offsets of the multi-bit fields in the XsRawGnssHeading::m_flags field
*/
enum XsGnssHeadingFlagOffset
{
	XGHFO_CarrSoln	= 3, //!< Bit offset of the field masked by XGHF_CarrSolnMask
};

/*! \enum XsGnssHeadingCarrierSolution
 *  \brief Defines the values of the carrier phase range solution field of XsRawGnssHeading::m_flags
*/
enum XsGnssHeadingCarrierSolution
{
	XGHCS_None	= 0, //!< No carrier phase range solution
	XGHCS_Float	= 1, //!< Carrier phase range solution with floating ambiguities
	XGHCS_Fixed	= 2, //!< Carrier phase range solution with fixed ambiguities
};

/*! @} */

/*! \brief A container for GNSS Heading data
*/
struct XsRawGnssHeading
{
	uint32_t	m_itow;				//!< GNSS time of week (ms)
	int32_t		m_relPosN;			//!< North component of the relative position vector from antenna #1 to antenna #2 in mm
	int32_t		m_relPosE;			//!< East component of the relative position vector from antenna #1 to antenna #2 in mm
	int32_t		m_relPosD;			//!< Down component of the relative position vector from antenna #1 to antenna #2 in mm
	int32_t		m_relPosLength;		//!< Length of the relative position vector from antenna #1 to antenna #2 in mm
	int32_t		m_relPosHeading;	//!< Angle between True North and baseline vector projected onto local horizontal plane in 1e-5 degrees
	uint32_t	m_accN;				//!< Accuracy of the relative position vector North component in mm
	uint32_t	m_accE;				//!< Accuracy of the relative position vector East component in mm
	uint32_t	m_accD;				//!< Accuracy of the relative position vector Down component in mm
	uint32_t	m_accLength;		//!< Accuracy of the relative position vector length in mm
	uint32_t	m_accHeading;		//!< Accuracy of the relative position vector heading in 1e-5 degrees
	uint32_t	m_flags;			//!< GNSS Heading status flags

#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	friend inline bool operator == (const XsRawGnssHeading& lhs, const XsRawGnssHeading& rhs)
	{
		// direct memory comparison is allowed because all fields line up properly
		return std::memcmp(&lhs, &rhs, sizeof(XsRawGnssHeading)) == 0;
	}
#endif

#ifdef SWIG
};
#else
} XS_PACKED_STRUCT;
#endif
typedef struct XsRawGnssHeading XsRawGnssHeading;

#ifdef _MSC_VER
#pragma pack(pop)
#elif defined (__ICCARM__)
_Pragma("pack(pop)")
#endif

#endif
