
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

#ifndef XSFLOATVECTOR3_H
#define XSFLOATVECTOR3_H

#ifdef __cplusplus
	#include "xstypedefs.h"
#endif
#ifndef __cplusplus
	#define XSFLOATVECTOR3_INITIALIZER {{0,0,0}}
#endif


/*! \brief A vector containing 3 float values. */
struct XsFloatVector3
{
#ifdef __cplusplus
	//! \brief Constructor that creates the vector with all components set to 0
	inline XsFloatVector3()
	{
		m_data[0] = 0;
		m_data[1] = 0;
		m_data[2] = 0;
	}

	//! \brief Constructor that creates the vector with all components set to given values \a v1 \a v2 and \a v3
	inline XsFloatVector3(float v1, float v2, float v3)
	{
		m_data[0] = v1;
		m_data[1] = v2;
		m_data[2] = v3;
	}

	//! \brief Constructor that creates the vector with all components set to values in array \a a
	inline explicit XsFloatVector3(const float* a)
	{
		m_data[0] = a[0];
		m_data[1] = a[1];
		m_data[2] = a[2];
	}

	//! \brief Constructor that creates the vector and initializes it with data from the \a other vector
	inline XsFloatVector3(const XsFloatVector3& other)
	{
		m_data[0] = other.m_data[0];
		m_data[1] = other.m_data[1];
		m_data[2] = other.m_data[2];
	}

	//! \brief Assignment operator copies the data from the \a other vector to this vector
	inline XsFloatVector3& operator = (const XsFloatVector3& other)
	{
		if (this != &other)
		{
			m_data[0] = other.m_data[0];
			m_data[1] = other.m_data[1];
			m_data[2] = other.m_data[2];
		}
		return *this;
	}

	//! \brief Comparison operator, returns true if the contents of the \a other vector match those of this vector
	inline bool operator == (const XsFloatVector3& other) const
	{
		return	m_data[0] == other.m_data[0] &&
			m_data[1] == other.m_data[1] &&
			m_data[2] == other.m_data[2];
	}

	//! \brief Return the size of the vector (always 3)
	inline XsSize size() const
	{
		return 3;
	}

	//! \brief Return a value from the vector (needed to allow generated C# access to these elements)
	inline float at(int index) const
	{
		return m_data[index];
	}

	//! \brief Returns the \a index'th item in the vector
	inline float operator[](XsSize index) const
	{
		assert(index < 3);
		return m_data[index];
	}

	//! \brief Returns a reference the \a index'th item in the vector
	inline float& operator[](XsSize index)
	{
		assert(index < 3);
		return m_data[index];
	}

	//! \brief Returns the start of the internal data buffer
	inline float const* data() const
	{
		return m_data;
	}
private:
#endif

	float m_data[3];	//!< vector component storage
};

typedef struct XsFloatVector3 XsFloatVector3;

#endif
