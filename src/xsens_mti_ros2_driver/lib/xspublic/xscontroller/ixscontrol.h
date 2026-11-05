
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

#ifndef IXSCONTROL_H
#define IXSCONTROL_H

#include <xstypes/pstdint.h>
#include <xstypes/xstypedefs.h>
#include <xstypes/xsexception.h>
#include <xstypes/xsstring.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsbaud.h>
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsintarray.h>
#include <xstypes/xssyncsettingarray.h>
#include <xstypes/xsfilepos.h>
#include <xstypes/xsfilterprofilearray.h>
#include <xstypes/xsoption.h>
#include <xscontroller/xsdeviceconfiguration.h>
#include <xscontroller/xsdeviceptrarray.h>
#include <xscontroller/xscallbackplainc.h>
// Defines XSENS_NO_PORT_NUMBERS on every non-Windows target. Included explicitly rather than
// relying on the build's force-include, because clang-autogen parses this header standalone:
// without it the generator would emit the port-number overloads into the C ABI on platforms
// whose concrete XsControl does not implement them.
#include <xscontroller/xscontrollerconfig.h>

struct XsControl;
struct XsDevice;

//! \brief Pure-abstract interface for XsControl, hand-maintained.
//! clang-autogen reads it to emit the C ABI and the C++ wrapper, which is why this header
//! uses plain includes and declares no method the concrete does not implement.
//!
//! This interface IS the public API of XsControl. A method that is public on the concrete but
//! not declared here is deliberately NOT exported. Widening the public surface is therefore an
//! explicit edit to this file, never a side effect of adding a method to the concrete.
//!
//! The concrete derives from this, so the compiler enforces the other direction: every
//! virtual declared here must exist on XsControl. That does not extend to the static members
//! below, which are matched by name at generation time instead.
//! scripts/clang_autogen_check.py reports both directions on demand, for when you want to
//! see the difference.
struct IXsControl
{
	virtual void flushInputBuffers() = 0;
	virtual void clearHardwareError() = 0;
	virtual void close() = 0;
	virtual bool openPort(const XsString& portname, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false) = 0;
	virtual bool openPort(XsPortInfo& portinfo, uint32_t timeout = 0, bool detectRs485 = false) = 0;
	virtual bool openPortWithCredentials(XsPortInfo& portinfo, XsString const& id, XsString const& key, uint32_t timeout = 0) = 0;
	virtual bool openCustomPort(int channelId, uint32_t channelLatency, bool detectRs485 = false) = 0;
	virtual bool openImarPort_internal(const XsString& portname, XsBaudRate baudrate, int imarType, uint32_t timeout = 0) = 0;
#ifndef XSENS_NO_PORT_NUMBERS
	virtual bool openPort(int portNr, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false) = 0;
#endif
	virtual void closePort(const XsString& portname) = 0;
	virtual void closePort(const XsDeviceId& deviceId) = 0;
	virtual void closePort(const XsPortInfo& portinfo) = 0;
	virtual void closeCustomPort(int channelId) = 0;
#ifndef XSENS_NO_PORT_NUMBERS
	virtual void closePort(int portNr) = 0;
#endif
	virtual void closePort(XsDevice* device) = 0;
	virtual XsPortInfo customPortInfo(int channelId) const = 0;
	virtual bool openLogFile(const XsString& filename) = 0;
	virtual XsResultValue lastResult() const = 0;
	virtual XsString lastResultText() const = 0;
	virtual XsResultValue lastHardwareError() const = 0;
	virtual XsDeviceId lastHardwareErrorDeviceId() const = 0;
	virtual int deviceCount() const = 0;
	virtual int mainDeviceCount() const = 0;
	virtual std::vector<XsDeviceId> mainDeviceIds() const = 0;
	virtual int mtCount() const = 0;
	virtual std::vector<XsDeviceId> mtDeviceIds() const = 0;
	virtual std::vector<XsDeviceId> deviceIds() const = 0;
	virtual XsDevice* getDeviceFromLocationId(uint16_t locationId) const = 0;
	virtual XsDeviceId dockDeviceId(const XsDeviceId& deviceId) const = 0;
	virtual bool isDeviceWireless(const XsDeviceId& deviceId) const = 0;
	virtual bool isDeviceDocked(const XsDeviceId& deviceId) const = 0;
	virtual bool loadFilterProfiles(const XsString& filename) = 0;
	virtual XsOption enabledOptions() const = 0;
	virtual XsOption disabledOptions() const = 0;
	virtual void setOptions(XsOption enable, XsOption disable) = 0;
	virtual void setOptionsForce(XsOption enabled) = 0;
	virtual bool setInitialPositionLLA(const XsVector& lla) = 0;
	virtual XsDevice* device(const XsDeviceId& deviceId) const = 0;
	virtual XsDevicePtrArray mainDevices() const = 0;
	virtual XsDevice* broadcast() const = 0;
	virtual void transmissionReceived(int channelId, const XsByteArray& data) = 0;
	virtual void gotoConfig() = 0;
	virtual void gotoMeasurement() = 0;
	virtual XsResultValue startRestoreCommunication(const XsString& portName) = 0;
	virtual void stopRestoreCommunication() = 0;


	// Implemented by the sibling CallbackManagerXda base, which cannot override across bases,
	// so the concrete forwards these explicitly.
	virtual void clearCallbackHandlers(bool chain = true) = 0;
	virtual void addCallbackHandler(XsCallbackPlainC* cb, bool chain = true) = 0;
	virtual void removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true) = 0;

	// Statics are clang-autogen's manifest, not part of the interface's contract: nothing
	// overrides a static, so the compiler never checks this against the concrete, and it is
	// defined nowhere here -- the generated C ABI qualifies it on XsControl. XSENS_CLANG_AUTOGEN
	// is defined only for that parse (scripts/clang_autogen.py base_args), so it reaches the ABI
	// while no compiler that builds the product can name IXsControl::resultText() and get a
	// symbol nothing defines. Grouped last because the exported C name of an overload carries its
	// position within its own name group; a name declared once, like this one, does not move.
#ifdef XSENS_CLANG_AUTOGEN
	static XsString resultText(XsResultValue resultCode);
#endif

	virtual ~IXsControl() {}
};

#endif // IXSCONTROL_H
