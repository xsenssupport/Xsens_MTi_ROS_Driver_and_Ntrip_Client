
//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
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

#ifndef XSVECTOR2_H
#define XSVECTOR2_H

#include "xsvector.h"

struct XsVector2;
#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
typedef struct XsVector2 XsVector2;
#endif

XSTYPES_DLL_API void XsVector2_construct(XsVector2* thisPtr, const XsReal* src);
XSTYPES_DLL_API void XsVector2_assign(XsVector2* thisPtr, const XsReal* src);
XSTYPES_DLL_API void XsVector2_destruct(XsVector2* thisPtr);
XSTYPES_DLL_API void XsVector2_copy(XsVector* copy, XsVector2 const* src);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
/*  This is allowed since the C standard says that no padding appears before the first member of a struct.
	Basically we're defining a union between a C++ inherited type and a C encapsulated type.
*/
struct XsVector2 : public XsVector
{
	XSCPPPROTECTED
#else
struct XsVector2
{
	XsVector m_vector;		//!< The underlying vector
#endif
	XsReal m_fixedData[2];				//!< Fixed size storage for the components in the vector

#ifdef __cplusplus
public:
	//! \brief Constructs an empty vector2
	XsVector2() : XsVector(m_fixedData, 2, XSDF_FixedSize)
	{
		//XsVector2_construct(this, 0);
	}

	//! \brief Constructs a vector2 from an \a other XsVector
	XsVector2(XsVector2 const& other) : XsVector(other, m_fixedData, 2, XSDF_FixedSize)
	{
	}

	//! \brief Constructs a vector2 from an \a other XsVector
	XsVector2(XsVector const& other) : XsVector(other, m_fixedData, 2, XSDF_FixedSize)
	{
	}

	//! \brief Constructs a vector2 using the values \a x, \a y, \a z
	XsVector2(XsReal x, XsReal y) : XsVector(m_fixedData, 2, XSDF_FixedSize)
	{
		m_data[0] = x;
		m_data[1] = y;
	}

	//! \brief Return a 2-element zero vector
	static XsVector2 const& zero2()
	{
		static const XsVector2 rv(XsMath_zero, XsMath_zero);
		return rv;
	}

	using XsVector::operator=;
	//! \brief Assignment operator. Copies from \a other into this
	inline XsVector2& operator=(const XsVector2& other)
	{
		XsVector_copy(this, &other);
		return *this;
	}

	using XsVector::assign;
	//! \brief Assignment, directly assigns values.
	inline void assign(XsReal x, XsReal y)
	{
		m_data[0] = x;
		m_data[1] = y;
		const_cast<XsSize&>(m_flags) &= ~XSDF_Empty;
	}
	//	using XsVector::operator[];
#endif
};

#endif
