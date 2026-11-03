
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

#ifndef IXSDEVICE_H
#define IXSDEVICE_H

#include <xstypes/xsstring.h>
#include <xstypes/xsdeviceid.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsversion.h>
#include <xstypes/xsscrdata.h>
#include <xstypes/xscalibrateddata.h>
#include <xstypes/xstimeinfo.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xscanoutputconfigurationarray.h>
#include <xstypes/xsintarray.h>
#include <xstypes/xsmatrix3x3.h>
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xssyncsettingarray.h>
#include <xstypes/xsmatrix.h>
#include <xstypes/xsvector.h>
#include <xstypes/xsstringarray.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsfilterprofilearray.h>
#include <xstypes/xsoutputconfiguration.h>
#include <xstypes/xsstringoutputtypearray.h>
#include <xstypes/xsexception.h>
#include <xscontroller/xscallbackplainc.h>
#include <xscontroller/xsselftestresult.h>
#include <xscontroller/xsdeviceparameter.h>
#include <xscontroller/xsiccrepmotionresult.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsbaud.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xssyncrole.h>
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsfilepos.h>
#include <xstypes/xsdeviceoptionflag.h>
#include <xstypes/xsresetmethod.h>
#include <xstypes/xsoption.h>
#include <xscontroller/xsdeviceptrarray.h>
#include <xscontroller/xsdevicestate.h>
#include <xscontroller/xserrormode.h>
#include <xscontroller/xsdeviceconfiguration.h>
#include <xscontroller/xsprotocoltype.h>
#include <xscontroller/xsrejectreason.h>
#include <xscontroller/xsalignmentframe.h>
#include <xscontroller/xsoperationalmode.h>
#include <xscontroller/xsaccesscontrolmode.h>
#include <xscontroller/xsubloxgnssplatform.h>
#include <xscontroller/xsleverarmtype.h>
#include <xscontroller/xsdualantennamode.h>
// Defines XSENS_NO_PORT_NUMBERS on every non-Windows target. Included explicitly rather than
// relying on the build's force-include, because clang-autogen parses this header standalone:
// without it the generator would emit portNumber() into the C ABI on platforms whose concrete
// XsDevice does not implement it.
#include <xscontroller/xscontrollerconfig.h>

struct XsDevice;

//! \brief Pure-abstract interface for XsDevice, hand-maintained.
//! clang-autogen reads it to emit the C ABI and the C++ wrapper, which is why this header
//! uses plain includes and declares no method the concrete does not implement.
//!
//! This interface IS the public API of XsDevice. A method that is public on the concrete but
//! not declared here is deliberately NOT exported. Widening the public surface is therefore an
//! explicit edit to this file, never a side effect of adding a method to the concrete.
//!
//! The concrete derives from this, so the compiler enforces the other direction: everything
//! declared here must exist on XsDevice. scripts/clang_autogen_check.py reports both
//! directions on demand, for when you want to see the difference.
struct IXsDevice
{
	virtual void addRef() = 0;
	virtual void removeRef() = 0;
	virtual XsSize refCounter() const = 0;
	virtual XsDevice* master() const = 0;
	virtual XsDevice* findDevice(XsDeviceId const& deviceid) const = 0;
	virtual XsDevice const* findDeviceConst(XsDeviceId const& deviceid) const = 0;
	virtual XsDevice* subDevice(int subDeviceId) const = 0;
	virtual int subDeviceCount() const = 0;
	virtual int busId() const = 0;
	virtual XsDeviceId const& deviceId() const = 0;
	virtual XsVersion firmwareVersion() const = 0;
	virtual bool isMasterDevice() const = 0;
	virtual bool isContainerDevice() const = 0;
	virtual bool isInitialized() const = 0;
	virtual bool isStandaloneDevice() const = 0;
	virtual const XsDevice* deviceAtBusIdConst(int busid) const = 0;
	virtual XsDevice* deviceAtBusId(int busid) = 0;
	virtual XsResultValue updatePortInfo(XsPortInfo const& newInfo) = 0;
	virtual void setGotoConfigOnClose(bool gotoConfigOnClose) = 0;
	virtual XsResultValue createLogFile(const XsString& filename) = 0;
	virtual bool closeLogFile() = 0;
	virtual bool isMeasuring() const = 0;
	virtual bool isRecording() const = 0;
	virtual bool isReadingFromFile() const = 0;
	virtual void restartFilter() = 0;
	virtual XsResultValue lastResult() const = 0;
	virtual XsString lastResultText() const = 0;
	virtual int recordingQueueLength() const = 0;
	virtual int cacheSize() const = 0;
	virtual XsDeviceState deviceState() const = 0;
	virtual XsDeviceConfiguration deviceConfiguration() const = 0;
	virtual int batteryLevel() const = 0;
	virtual int secondaryBatteryLevel() const = 0;
	virtual int updateRateForDataIdentifier(XsDataIdentifier dataType) const = 0;
	virtual int updateRateForProcessedDataIdentifier(XsDataIdentifier dataType) const = 0;
	virtual std::vector<int> supportedUpdateRates(XsDataIdentifier dataType = XDI_None) const = 0;
	virtual int maximumUpdateRate() const = 0;
	virtual bool hasDataEnabled(XsDataIdentifier dataType) const = 0;
	virtual bool hasProcessedDataEnabled(XsDataIdentifier dataType) const = 0;
	virtual XsString productCode() const = 0;
	virtual XsString shortProductCode() const = 0;
	virtual XsString portName() const = 0;
	virtual XsPortInfo portInfo() const = 0;
	virtual XsBaudRate baudRate() const = 0;
	virtual XsBaudRate serialBaudRate() const = 0;
	virtual XsVersion hardwareVersion() const = 0;
	virtual XsString hardwareId() const = 0;
#ifndef XSENS_NO_PORT_NUMBERS
	virtual int portNumber() const = 0;
#endif
	virtual bool startRecording() = 0;
	virtual bool triggerStartRecording() = 0;
	virtual bool stopRecording() = 0;
	virtual int64_t getStartRecordingPacketId() const = 0;
	virtual int64_t getStopRecordingPacketId() const = 0;
	virtual void setOptions(XsOption enable, XsOption disable) = 0;
	virtual bool areOptionsEnabled(XsOption options) const = 0;
	virtual XsOption getOptions() const = 0;
	virtual bool sendCustomMessage(const XsMessage& messageSend, bool waitForResult, XsMessage& messageReceive, int timeout = 0) = 0;
	virtual bool sendRawMessage(const XsMessage& message) = 0;
	virtual bool setSerialBaudRate(XsBaudRate baudrate) = 0;
	virtual XsIntArray portConfiguration() const = 0;
	virtual bool setPortConfiguration(XsIntArray& config) = 0;
	virtual bool isMotionTracker() const = 0;
	virtual XsOperationalMode operationalMode() const = 0;
	virtual bool setOperationalMode(XsOperationalMode mode) = 0;
	virtual int updateRate() const = 0;
	virtual bool setUpdateRate(int rate) = 0;
	virtual XsDeviceOptionFlag deviceOptionFlags() const = 0;
	virtual bool setDeviceOptionFlags(XsDeviceOptionFlag setFlags, XsDeviceOptionFlag clearFlags) = 0;
	virtual XsOutputConfigurationArray outputConfiguration() const = 0;
	virtual XsOutputConfigurationArray processedOutputConfiguration() const = 0;
	virtual bool setOutputConfiguration(XsOutputConfigurationArray& config) = 0;
	virtual bool isInStringOutputMode() const = 0;
	virtual XsCanOutputConfigurationArray canOutputConfiguration() const = 0;
	virtual bool setCanOutputConfiguration(XsCanOutputConfigurationArray& config) = 0;
	virtual uint32_t canConfiguration() const = 0;
	virtual bool setCanConfiguration(uint32_t config) = 0;
	virtual bool usesLegacyDeviceMode() const = 0;
	virtual uint16_t stringOutputType() const = 0;
	virtual uint16_t stringSamplePeriod() const = 0;
	virtual uint16_t stringSkipFactor() const = 0;
	virtual bool setStringOutputMode(uint16_t type, uint16_t period, uint16_t skipFactor) = 0;
	virtual XsStringOutputTypeArray supportedStringOutputTypes() const = 0;
	virtual int dataLength() const = 0;
	virtual XsSyncSettingArray syncSettings() const = 0;
	virtual bool setSyncSettings(const XsSyncSettingArray& settingList) = 0;
	virtual bool isSyncMaster() const = 0;
	virtual bool isSyncSlave() const = 0;
	virtual XsSyncSettingArray supportedSyncSettings() const = 0;
	virtual bool gotoMeasurement() = 0;
	virtual bool gotoConfig() = 0;
	virtual bool restoreFactoryDefaults() = 0;
	virtual bool reset() = 0;
	virtual bool reopenPort(bool gotoConfig, bool skipDeviceIdCheck = false) = 0;
	virtual void writeDeviceSettingsToFile() = 0;
	virtual void flushInputBuffers() = 0;
	virtual XsSyncRole syncRole() const = 0;
	virtual bool loadLogFile() = 0;
	virtual bool abortLoadLogFile() = 0;
	virtual XsString logFileName() const = 0;
	virtual bool resetOrientation(XsResetMethod resetmethod) = 0;
	virtual bool resetLogFileReadPosition() = 0;
	virtual XsFilePos logFileSize() const = 0;
	virtual XsFilePos logFileReadPosition() const = 0;
	virtual bool updateCachedDeviceInformation() = 0;
	virtual bool enableProtocol(XsProtocolType protocol) = 0;
	virtual bool disableProtocol(XsProtocolType protocol) = 0;
	virtual bool isProtocolEnabled(XsProtocolType protocol) const = 0;
	virtual uint32_t deviceBufferSize() = 0;
	virtual bool setDeviceBufferSize(uint32_t frames) = 0;
	virtual XsConnectivityState connectivityState() const = 0;
	virtual void waitForAllDevicesInitialized() = 0;
	virtual std::vector<XsDevice*> children() const = 0;
	virtual int childCount() const = 0;
	virtual bool enableRadio(int channel) = 0;
	virtual bool disableRadio() = 0;
	virtual int radioChannel() const = 0;
	virtual bool isRadioEnabled() const = 0;
	virtual bool makeOperational() = 0;
	virtual bool isOperational() const = 0;
	virtual bool isInSyncStationMode() = 0;
	virtual bool setSyncStationMode(bool enabled) = 0;
	virtual bool stealthMode() const = 0;
	virtual bool setStealthMode(bool enabled) = 0;
	virtual void discardRetransmissions(int64_t firstNewPacketId) = 0;
	virtual bool abortFlushing() = 0;
	virtual bool setDeviceAccepted(const XsDeviceId& deviceId) = 0;
	virtual bool setDeviceRejected(const XsDeviceId& deviceId) = 0;
	virtual bool setAccessControlMode(XsAccessControlMode mode, const XsDeviceIdArray& initialList) = 0;
	virtual XsAccessControlMode accessControlMode() const = 0;
	virtual XsDeviceIdArray currentAccessControlList() const = 0;
	virtual XsResultValue setDeviceParameter(XsDeviceParameter const& parameter) = 0;
	virtual XsResultValue deviceParameter(XsDeviceParameter& parameter) const = 0;
	virtual XsUbloxGnssPlatform ubloxGnssPlatform() const = 0;
	virtual bool setUbloxGnssPlatform(XsUbloxGnssPlatform ubloxGnssPlatform) = 0;
	virtual XsIntArray gnssReceiverSettings() const = 0;
	virtual bool setGnssReceiverSettings(const XsIntArray& gnssReceiverSettings) = 0;
	virtual bool acceptConnection() = 0;
	virtual bool rejectConnection() = 0;
	virtual int wirelessPriority() const = 0;
	virtual bool setWirelessPriority(int priority) = 0;
	virtual XsRejectReason rejectReason() const = 0;
	virtual bool requestBatteryLevel() = 0;
	virtual XsTimeStamp batteryLevelTime() = 0;
	virtual bool setTransportMode(bool transportModeEnabled) = 0;
	virtual bool transportMode() = 0;
	virtual int16_t lastKnownRssi() const = 0;
	virtual int packetErrorRate() const = 0;
	virtual bool isBlueToothEnabled() const = 0;
	virtual bool setBlueToothEnabled(bool enabled) = 0;
	virtual bool isBusPowerEnabled() const = 0;
	virtual bool setBusPowerEnabled(bool enabled) = 0;
	virtual bool powerDown() = 0;
	virtual XsErrorMode errorMode() const = 0;
	virtual bool setErrorMode(XsErrorMode errormode) = 0;
	virtual bool setInitialHeading(double heading) = 0;
	virtual bool setHeadingOffset(double offset) = 0;
	virtual double headingOffset() const = 0;
	virtual bool setLocationId(int id) = 0;
	virtual int locationId() const = 0;
	virtual XsDevice* getDeviceFromLocationId(uint16_t locId) = 0;
	virtual XsMatrix objectAlignment() const = 0;
	virtual bool setObjectAlignment(const XsMatrix& matrix) = 0;
	virtual double gravityMagnitude() const = 0;
	virtual bool setGravityMagnitude(double mag) = 0;
	virtual XsVector initialPositionLLA() const = 0;
	virtual bool setInitialPositionLLA(const XsVector& lla) = 0;
	virtual XsTimeInfo utcTime() const = 0;
	virtual bool setUtcTime(const XsTimeInfo& time) = 0;
	virtual bool reinitialize() = 0;
	virtual XsFilterProfile xdaFilterProfile() const = 0;
	virtual bool setXdaFilterProfile(int profileType) = 0;
	virtual bool setXdaFilterProfile(XsString const& profileType) = 0;
	virtual XsFilterProfile onboardFilterProfile() const = 0;
	virtual bool setOnboardFilterProfile(int profileType) = 0;
	virtual bool setOnboardFilterProfile(XsString const& profileType) = 0;
	virtual bool replaceFilterProfile(XsFilterProfile const& profileCurrent, XsFilterProfile const& profileNew) = 0;
	virtual XsFilterProfileArray availableOnboardFilterProfiles() const = 0;
	virtual XsFilterProfileArray availableXdaFilterProfiles() const = 0;
	virtual double accelerometerRange() const = 0;
	virtual double actualAccelerometerRange() const = 0;
	virtual double gyroscopeRange() const = 0;
	virtual double actualGyroscopeRange() const = 0;
	virtual bool setNoRotation(uint16_t duration) = 0;
	virtual bool startRepresentativeMotion() = 0;
	virtual bool representativeMotionState() = 0;
	virtual XsIccRepMotionResult stopRepresentativeMotion() = 0;
	virtual XsIccRepMotionResult getIccState() = 0;
	virtual bool storeIccResults() = 0;
	virtual uint16_t rs485TransmissionDelay() const = 0;
	virtual bool setRs485TransmissionDelay(uint16_t delay) = 0;
	virtual XsSelfTestResult runSelfTest() = 0;
	virtual bool requestData() = 0;
	virtual bool storeFilterState() = 0;
	virtual uint16_t deadReckoningDuration() const = 0;
	virtual bool setDeadReckoningDuration(uint16_t seconds) = 0;
	virtual XsVector modelingParameters(uint8_t sensingElement) const = 0;
	virtual XsDataPacket getDataPacketByIndex(XsSize index) const = 0;
	virtual XsSize getDataPacketCount() const = 0;
	virtual XsDataPacket lastAvailableLiveData() const = 0;
	virtual XsDataPacket takeFirstDataPacketInQueue() = 0;
	virtual bool isInitialBiasUpdateEnabled() const = 0;
	virtual bool setInitialBiasUpdateEnabled(bool enable) = 0;
	virtual bool isFixedGravityEnabled() const = 0;
	virtual bool setFixedGravityEnabled(bool enable) = 0;
	virtual XsResultValue createConfigFile(const XsString& filename) = 0;
	virtual XsResultValue applyConfigFile(const XsString& filename) = 0;
	virtual bool setAlignmentRotationMatrix(XsAlignmentFrame frame, const XsMatrix& matrix) = 0;
	virtual XsMatrix alignmentRotationMatrix(XsAlignmentFrame frame) const = 0;
	virtual bool setAlignmentRotationQuaternion(XsAlignmentFrame frame, const XsQuaternion& quat) = 0;
	virtual XsQuaternion alignmentRotationQuaternion(XsAlignmentFrame frame) const = 0;
	virtual bool setLeverArm(XsLeverArmType type, const XsVector& arm) = 0;
	virtual XsVector leverArm(XsLeverArmType type) const = 0;
	virtual bool setDualAntennaSettings(XsDualAntennaMode mode, float angle = 0) = 0;
	virtual XsDualAntennaMode dualAntennaSettings(float* angle = nullptr) const = 0;
	virtual bool deviceIsDocked(XsDevice* dev) const = 0;
	virtual bool isLoadLogFileInProgress() const = 0;
	virtual void waitForLoadLogFileDone() const = 0;
	virtual bool setGnssLeverArm(const XsVector& arm) = 0;
	virtual XsVector gnssLeverArm() const = 0;
	virtual uint32_t supportedStatusFlags() const = 0;
	virtual XsByteArray readMetaDataFromLogFile() = 0;


	// Implemented by the sibling CallbackManagerXda base, which cannot override across bases,
	// so the concrete forwards these explicitly.
	virtual void clearCallbackHandlers(bool chain = true) = 0;
	virtual void addCallbackHandler(XsCallbackPlainC* cb, bool chain = true) = 0;
	virtual void removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true) = 0;

	// Statics are clang-autogen's manifest, not part of the interface's contract: nothing
	// overrides a static, so the compiler never checks these against the concrete, and they are
	// defined nowhere here -- the generated C ABI qualifies them on XsDevice. XSENS_CLANG_AUTOGEN
	// is defined only for that parse (scripts/clang_autogen.py base_args), so they reach the ABI
	// while no compiler that builds the product can name IXsDevice::supportsSyncSettings() and
	// get a symbol nothing defines.
	//
	// Safe to group here: an overload's exported C name carries its position within its own name
	// group, and supportedSyncSettings -- the only name with both a virtual and a static -- keeps
	// the static after the virtual either way.
#ifdef XSENS_CLANG_AUTOGEN
	static bool supportsSyncSettings(XsDeviceId const& deviceId);
	static bool isCompatibleSyncSetting(XsDeviceId const& deviceId, XsSyncSetting const& setting1, XsSyncSetting const& setting2);
	static unsigned int syncSettingsTimeResolutionInMicroSeconds(XsDeviceId const& deviceId);
	static XsSyncSettingArray supportedSyncSettings(XsDeviceId const& deviceId);
#endif

	virtual ~IXsDevice() {}
};

#endif // IXSDEVICE_H
