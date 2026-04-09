
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

#include "xsvector2.h"
#include <string.h>

/*! \class XsVector2
	\brief A class that represents a fixed size (2) vector
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVector2 \brief Init the %XsVector2 and copy the data from \a src into the vector if \a src is not null */
void XsVector2_construct(XsVector2* thisPtr, const XsReal* src)
{
	XsVector_ref(&thisPtr->m_vector, 2, (XsReal*) thisPtr->m_fixedData, XSDF_FixedSize);
	if (src)
		memcpy((XsReal*) thisPtr->m_fixedData, src, 2 * sizeof(XsReal));
}

/*! \relates XsVector2 \brief Init the %XsVector2 and copy the data from \a src into the vector if \a src is not null */
void XsVector2_assign(XsVector2* thisPtr, const XsReal* src)
{
	XsVector_assign(&thisPtr->m_vector, 2, src);
}

/*! \relates XsVector2 \brief Frees the XsVector2 */
void XsVector2_destruct(XsVector2* thisPtr)
{
	// don't do anything, no memory needs to be freed
	assert(thisPtr->m_vector.m_flags & XSDF_FixedSize);
	(void)thisPtr;
}

/*! \relates XsVector2 \brief Copy the contents of the %XsVector2 to \a copy */
void XsVector2_copy(XsVector* copy, XsVector2 const* src)
{
	XsVector_copy(copy, &src->m_vector);
}

/*! @} */
