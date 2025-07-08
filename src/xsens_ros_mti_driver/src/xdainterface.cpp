
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
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

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsfilterprofilearray.h>
#include <xstypes/xstypedefs.h>
#include <xstypes/xsdatapacket.h>
#include <mavros_msgs/RTCM.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/orientationeulerpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"
#include "messagepublishers/statuspublisher.h"
#include "messagepublishers/nmeapublisher.h"
#include "messagepublishers/gnssposepublisher.h"
#include "messagepublishers/utctimepublisher.h"
#include "messagepublishers/accelerationhrpublisher.h"
#include "messagepublishers/angularvelocityhrpublisher.h"
#include "xsens_log_handler.h"

#define XS_DEFAULT_BAUDRATE (115200)


XdaInterface::XdaInterface(ros::NodeHandle &node)
    : m_device(nullptr), m_node(node)
{
    ROS_INFO("Creating XsControl object...");
	m_productCode = "";
    m_control = XsControl::construct();
    assert(m_control != 0);
}



XdaInterface::~XdaInterface()
{
	ROS_INFO("Cleaning up ...");
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

void XdaInterface::registerPublishers()
{
	bool should_publish;
	bool isDeviceVruAhrs = m_device->deviceId().isAhrs() || m_device->deviceId().isVru();
	bool isDeviceGnss = m_device->deviceId().isGnss();
	bool isDeviceGnssRtk = m_device->deviceId().isRtk();


	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(m_node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(m_node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(m_node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(m_node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(m_node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(m_node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(m_node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(m_node));
	}
	if (ros::param::get("~pub_status", should_publish) && should_publish)
	{
		//ROS_INFO("registerCallback StatusPublisher....");
		registerCallback(new StatusPublisher(m_node));
	}
	if (ros::param::get("~pub_utctime", should_publish) && should_publish)
	{
		//ROS_INFO("registerCallback UTCTimePublisher....");
		registerCallback(new UTCTimePublisher(m_node));
	}
	if (ros::param::get("~pub_accelerationhr", should_publish) && should_publish)
	{
		registerCallback(new AccelerationHRPublisher(m_node));
	}
	if (ros::param::get("~pub_angular_velocity_hr", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityHRPublisher(m_node));
	}

	if(isDeviceVruAhrs || isDeviceGnss)
	{
		if (ros::param::get("~pub_imu", should_publish) && should_publish)
		{
			registerCallback(new ImuPublisher(m_node));
		}
		if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
		{
			registerCallback(new OrientationPublisher(m_node));
		}
		if (ros::param::get("~pub_euler", should_publish) && should_publish)
		{
			registerCallback(new OrientationEulerPublisher(m_node));
		}
		if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
		{
			registerCallback(new FreeAccelerationPublisher(m_node));
		}
		if (ros::param::get("~pub_transform", should_publish) && should_publish)
		{
			registerCallback(new TransformPublisher(m_node));
		}

	}
	

	if(isDeviceGnss)
	{
		if (ros::param::get("~pub_gnss", should_publish) && should_publish)
		{
			registerCallback(new GnssPublisher(m_node));
		}
		if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
		{
			registerCallback(new PositionLLAPublisher(m_node));
		}
		if (ros::param::get("~pub_velocity", should_publish) && should_publish)
		{
			registerCallback(new VelocityPublisher(m_node));
		}
		if (ros::param::get("~pub_twist", should_publish) && should_publish)
		{
			registerCallback(new TwistPublisher(m_node));
		}
		if (ros::param::get("~pub_nmea", should_publish) && should_publish)
		{
			//ROS_INFO("registerCallback NMEAPublisher....");
			registerCallback(new NMEAPublisher(m_node));
		}
		if (ros::param::get("~pub_gnsspose", should_publish) && should_publish)
		{
			//ROS_INFO("registerCallback GNSSPOSEPublisher....");
			registerCallback(new GNSSPOSEPublisher(m_node));
		}
	}

	//For RTK GNSS/INS MTi-8, MTi-680(G), we need to subscribe to a rostopic /rtcm.
	if(isDeviceGnssRtk)
	{
		m_node.subscribe<mavros_msgs::RTCM>("/rtcm", 100, boost::bind(&XdaInterface::rtcmCallback, this, _1));
		ROS_INFO("subscribing to /rtcm rostopic");
	}


}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;
	XsBaudRate baudrate = XBR_Invalid;
	bool checkDeviceID = false;
	std::string deviceId = "";
	
	// Check if scanning is enabled
	bool scan_for_devices = false;
	ros::param::get("~scan_for_devices", scan_for_devices);

	int timeout = 100; //100ms is default, but for Jetson Nano /dev/ttyTHS1, need to use a higher value.
	if (ros::param::has("~serial_timeout"))
	{
		ros::param::get("~serial_timeout", timeout);
		ROS_INFO("Found serial timeout parameter: %d", timeout);
	}

	if (!scan_for_devices)
	{
		// Read baudrate parameter if set
		if (ros::param::has("~baudrate"))
		{
			int baudrateParam = 0;
			ros::param::get("~baudrate", baudrateParam);
			ROS_INFO("Found baudrate parameter: %d", baudrateParam);
			baudrate = XsBaud::numericToRate(baudrateParam);
		}

		// Read port parameter if set
		if (ros::param::has("~port"))
		{
			std::string portName;
			ros::param::get("~port", portName);
			ROS_INFO("Found port name parameter: %s", portName.c_str());
			mtPort = XsPortInfo(portName, baudrate);
			ROS_INFO("Scanning port %s ...", portName.c_str());
			if (!XsScanner::scanPort(mtPort, baudrate, timeout))
				return handleError("No MTi device found. Verify port and baudrate.");

		}


	}
	else
	{
		ROS_INFO("Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate, timeout);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	// Read device ID parameter
	if (ros::param::has("~device_id"))
	{
		ros::param::get("~device_id", deviceId);
		checkDeviceID = true;
		ROS_INFO("Found device ID parameter: %s.", deviceId.c_str());
	}
	//If the device ID doesn't match the ID, throw error and exit program.
	if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
	return handleError("No MTi device found with matching device ID.");

	if (mtPort.empty())
		return handleError("No MTi device found.");

	ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	m_productCode = m_device->productCode();
	ROS_INFO("Device: %s, with ID: %s opened.", m_productCode.toStdString().c_str(), m_device->deviceId().toString().c_str());
	//print firmware version
	XsVersion firmwareVersion = m_device->firmwareVersion();
	ROS_INFO("Firmware version: %s", firmwareVersion.toString().c_str());
	//check if it is mti-670(G) or 680(G), if yes, and if the firmware version is less than 1.12.0, ROS_WARN to ask user to update firmware.
	bool isMTi670 = false;
	bool isMTi680 = false;
	isMTi670 = m_productCode.toStdString().substr(4, 3) == "670";
	isMTi680 = m_productCode.toStdString().substr(4, 3) == "680";
	if (isMTi680 || isMTi670)
	{
		if (firmwareVersion < XsVersion(1, 12, 0))
		{
			ROS_WARN("Firmware version is less than 1.12.0, please update firmware to 1.12.0 or later.");
		}
	}
	//get filter profile
	XsFilterProfile onBoardFilterProfile = m_device->onboardFilterProfile();
	ROS_INFO("Onboard Kalman Filter Option: %s", onBoardFilterProfile.toString().c_str());
	//get option flags like inrun compass, ahs
	XsDeviceOptionFlag optionFlags = m_device->deviceOptionFlags();
	if((optionFlags & XDOF_EnableAhs) == XDOF_EnableAhs)
	{
		ROS_INFO("Optionflag AHS is enabled.");
	}
	if((optionFlags & XDOF_EnableInrunCompassCalibration) == XDOF_EnableInrunCompassCalibration)
	{
		ROS_INFO("Optionflag InrunCompassCalibration is enabled.");
	}
	if((optionFlags & XDOF_EnableOrientationSmoother) == XDOF_EnableOrientationSmoother)
	{
		ROS_INFO("Optionflag OrientationSmoother is enabled.");
	}
	if((optionFlags & XDOF_EnablePositionVelocitySmoother) == XDOF_EnablePositionVelocitySmoother)
	{
		ROS_INFO("Optionflag PositionVelocitySmoother is enabled.");
	}
	if((optionFlags & XDOF_EnableContinuousZRU) == XDOF_EnableContinuousZRU)
	{
		ROS_INFO("Optionflag ContinuousZRU is enabled.");
	}
	if((optionFlags & XDOF_EnableBeidou) == XDOF_EnableBeidou)
	{
		ROS_INFO("Optionflag EnableBeidou is enabled.");
	}

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}


/**
 * \brief Prepares the device for operation by configuring it based on parameters and starting necessary services.
 * 
 * This method performs several steps to prepare the device for operation:
 * - Switches the device to configuration mode.
 * - Reads and applies GNSS lever arm settings if the device supports RTK and the parameter is provided.
 * - Configures the GNSS platform if the device supports GNSS and the parameter is provided.
 * - Reads the EMTS and device configuration stored in the .mtb file.
 * - Switches the device to measurement mode.
 * - Optionally starts logging data to a file if the parameter is set.
 * - Sends a manual gyro bias estimation command.
 * - Sets up a periodic manual gyro bias estimation if the parameter is provided.
 * 
 * \return Returns true if all preparation steps are successful, otherwise returns false and handles the error.
 */
bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	//configure sensor settings
	configureSensorSettings();

	int time_option = 0; //default is "mti_utc"
	ros::param::get("~time_option", time_option);
	if (time_option == 0)
	{
		ROS_INFO("Rosnode time_option is utc time from MTi");
	}
	else if (time_option == 1)
	{
		ROS_INFO("Rosnode time_option is sample time fine from MTi");
	}
	else
	{
		ROS_WARN("Rosnode time_option is using host controller's ros time, no recommended, use MT Manager - Device Settings - Output Configurations to select utc time or sample time fine, and set time_option to 0 or 1 in the xsens_mti_node.yaml file. ");
	}

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	ROS_INFO("Measuring ..");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	bool enable_logging = false;
	if (ros::param::get("~enable_logging", enable_logging) && enable_logging)
	{
		XsensLogHandler logHandler;
		logHandler.prepareLogDirectory();
		std::string log_file = logHandler.getLogFileName();

		if (m_device->createLogFile(log_file) != XRV_OK)
			return handleError(std::string("Failed to create a log file! (%s)") + log_file);
		else
			ROS_INFO("Created a log file: %s", log_file.c_str());

		if (!m_device->startRecording())
			return handleError("Could not start recording");
	}

	//delay 0.05 second, as the previous actions might take a little delay.
	ros::Duration(0.05).sleep();
	
	//in any case, send MGBE in the beginning for 6 seconds.
	manualGyroBiasEstimation(6);

	// Setup Periodic Manual Gyro Bias Estimation
        setupManualGyroBiasEstimation();

	return true;
}


/**
 * \brief Sets up the manual gyro bias estimation process based on ROS parameters.
 *
 * This function reads ROS parameters to determine if manual gyro bias estimation (MGBE) should be enabled
 * and retrieves the necessary parameters for the estimation process. If enabled and valid parameters are provided,
 * it sets up a timer to periodically initiate the MGBE process. The function checks for the 'enable_manual_gyro_bias'
 * parameter to decide whether the MGBE should be enabled, and 'manual_gyro_bias_param' for the parameters required
 * for the estimation process, which includes the event interval and duration for the MGBE.
 */
bool XdaInterface::manualGyroBiasEstimation(uint16_t duration)
{
    // Check if duration is less than 2; if so, set it to 2
    if (duration < 2)
    {
        ROS_INFO("Duration is less than 2 seconds, setting it to 2 seconds.");
        duration = 2;
    }

    XsMessage snd(XMID_SetNoRotation, sizeof(uint16_t));
    XsMessage rcv;
    snd.setDataShort(duration);
    if (!m_device->sendCustomMessage(snd, true, rcv, 1000))
        return false;

    //ROS_INFO("Manual Gyro Bias Estimation sent at %d seconds.", duration);
    return true;
}



/**
 * \brief Sets up the manual gyro bias estimation process based on ROS parameters.
 *
 * This function reads ROS parameters to determine if manual gyro bias estimation (MGBE) should be enabled
 * and retrieves the necessary parameters for the estimation process. If enabled and valid parameters are provided,
 * it sets up a timer to periodically initiate the MGBE process. The function checks for the 'enable_manual_gyro_bias'
 * parameter to decide whether the MGBE should be enabled, and 'manual_gyro_bias_param' for the parameters required
 * for the estimation process, which includes the event interval and duration for the MGBE.
 */

void XdaInterface::setupManualGyroBiasEstimation()
{
    // Check if manual gyro bias estimation is enabled and parameters are available
    bool enable_manual_gyro_bias;
    std::vector<int> manual_gyro_bias_param;
    ros::param::get("~enable_manual_gyro_bias", enable_manual_gyro_bias);
    ros::param::get("~manual_gyro_bias_param", manual_gyro_bias_param);
    if (enable_manual_gyro_bias)
    {
        if (manual_gyro_bias_param.size() == 2)
        {
            int event_interval = manual_gyro_bias_param[0];
            int duration = manual_gyro_bias_param[1];

            if (event_interval < 10)
            {
                ROS_INFO("Event interval is less than 10 seconds, setting it to 10 seconds.");
                event_interval = 10;
            }
	    if (duration < 2)
	    {
	        ROS_INFO("Duration is less than 2 seconds, setting it to 2 seconds.");
	        duration = 2;
	    }

            // Start the timer for MGBE with the retrieved parameters
            m_manualGyroBiasTimer = m_node.createTimer(ros::Duration(event_interval), [this, duration](const ros::TimerEvent&){
                this->manualGyroBiasEstimation(duration);
            });

            ROS_INFO("Manual Gyro Bias Estimation enabled. Interval: %d seconds, Duration: %d seconds.", event_interval, duration);
	    event_interval += duration;
        }
        else
        {
            ROS_ERROR("Invalid or missing 'manual_gyro_bias_param'. MGBE will not be performed.");
        }
    }
    else
    {
        ROS_INFO("Manual Gyro Bias Estimation is disabled.");
    }
}


/**
 * \brief Callback function to handle RTCM messages received from NTRIP Client.
 *
 * This function is designed to be called whenever a new RTCM message is received. It constructs an XsMessage
 * with the RTCM data and forwards it via MTi to the GNSS receiver. 
 *
 * \param msg The RTCM message received from NTRIP Client, encapsulated in a ROS message pointer. This message
 *            contains the RTCM data that needs to be forwarded to the GNSS receiver.
 */
void XdaInterface::rtcmCallback(const mavros_msgs::RTCM::ConstPtr &msg)
{
	XsMessage rtcm(XMID_ForwardGnssData);
	uint16_t rtcmMessageLength = (const uint16_t)msg->data.size();
	rtcm.setDataBuffer((const uint8_t *)&msg->data[0], rtcmMessageLength, 0);
	XsMessage rcv;
	m_device->sendCustomMessage(rtcm, false, rcv, 0);
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	ROS_ERROR("%s", error.c_str());
	return false;
}


/**
 * \brief Configures the sensor settings based on ROS parameters.
 *
 * \return Returns true if the sensor was successfully configured; otherwise, it
 *         returns false if it encounters any error during the configuration process.
 */
bool XdaInterface::configureSensorSettings()
{
	assert(m_device != 0);

	if(!ros::param::has("~enable_deviceConfig"))
		return handleError("No enable_deviceConfig found in xsens_mti_node.yaml.");

	bool enable_deviceConfig = false;
	ros::param::get("~enable_deviceConfig", enable_deviceConfig);
	if (enable_deviceConfig == false)
	{
		ROS_INFO("enable_deviceConfig is false, no need to configure MTI.");
		return false;
	}

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");
	
	if (enable_deviceConfig)
	{	

		XsVersion firmwareVersion = m_device->firmwareVersion();
		bool isDeviceGnssIns = m_device->deviceId().isGnss();
		bool isDeviceVruAhrs = m_device->deviceId().isAhrs() || m_device->deviceId().isVru();
		bool isMTiX = m_device->deviceId().isMtiX(); // check if it is MTi-1/2/3/7/8

		//use ros param get to get output_data_rate
		int ODRoption = 100;
		if(!isMTiX)
		{
			if(ros::param::get("~output_data_rate", ODRoption))
			{
				if(ODRoption <= 400)
				{
					ROS_INFO("Got output_data_rate param at %dHz", ODRoption);
				}
				else
				{
					ROS_WARN("output_data_rate is got but set over 400Hz, using default 100Hz");
				}
				
			}
			else
			{
				ROS_WARN("Cant't get output_data_rate param, using default 100Hz");
			}
		}

		// use ros param to get output_data_rate_lower
		int ODRoptionLower = 100; //for mag, baro, or MTi-1 series, max 100Hz.
		if(ros::param::get("~output_data_rate_lower", ODRoptionLower))
		{
			if(ODRoptionLower <=100)
			{
				ROS_INFO("Got output_data_rate_lower param at %dHz", ODRoptionLower);
			}
			else
			{
				ROS_WARN("output_data_rate_lower is got but set too high, using default 100Hz");
			}
		}
		else
		{
			ROS_WARN("Cant't get output_data_rate_lower param, using default 100Hz");
		}



		ROS_INFO("Configuring sensor output...");
		XsOutputConfigurationArray configArray;
		//for theses output, always output.
		configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
		configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
		configArray.push_back(XsOutputConfiguration(XDI_UtcTime, 0));
		//ROS_INFO print the config names and frequencies
		ROS_INFO("XDI_PacketCounter, XDI_SampleTimeFine, XDI_UtcTime");
		//use ros param to check pub_utctime, if yes, then push back it
		bool should_config = false;
		//for these data, all mti series have them.
		if (ros::param::get("~pub_acceleration", should_config) && should_config)
		{
			if(isMTiX)
			{
				configArray.push_back(XsOutputConfiguration(XDI_Acceleration, ODRoptionLower));
				ROS_INFO("XDI_Acceleration, %dHz", ODRoptionLower);
			}
			else
			{
				configArray.push_back(XsOutputConfiguration(XDI_Acceleration, ODRoption));
				ROS_INFO("XDI_Acceleration, %dHz", ODRoption);
			}
		}
		if (ros::param::get("~pub_dv", should_config) && should_config)
		{
			if(isMTiX)
			{
				configArray.push_back(XsOutputConfiguration(XDI_DeltaV, ODRoptionLower));
				ROS_INFO("XDI_DeltaV, %dHz", ODRoptionLower);
			}
			else
			{
				configArray.push_back(XsOutputConfiguration(XDI_DeltaV, ODRoption));
				ROS_INFO("XDI_DeltaV, %dHz", ODRoption);
			}
		}
		if (ros::param::get("~pub_angular_velocity", should_config) && should_config)
		{
			if(isMTiX)
			{
				configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, ODRoptionLower));
				ROS_INFO("XDI_RateOfTurn, %dHz", ODRoptionLower);
			}
			else
			{
				configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, ODRoption));
				ROS_INFO("XDI_RateOfTurn, %dHz", ODRoption);
			}
		}
		if(ros::param::get("~pub_dq", should_config) && should_config)
		{
			if(isMTiX)
			{
				configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, ODRoptionLower));
				ROS_INFO("XDI_DeltaQ, %dHz", ODRoptionLower);
			}
			else
			{
				configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, ODRoption));
				ROS_INFO("XDI_DeltaQ, %dHz", ODRoption);
			}
		}
		if (ros::param::get("~pub_mag", should_config) && should_config)
		{
			configArray.push_back(XsOutputConfiguration(XDI_MagneticField, ODRoptionLower));
			ROS_INFO("XDI_MagneticField, %dHz", ODRoptionLower);
		}
		if (ros::param::get("~pub_temperature", should_config) && should_config)
		{
			if(isMTiX)
			{
				configArray.push_back(XsOutputConfiguration(XDI_Temperature, ODRoptionLower));
				ROS_INFO("XDI_Temperature, %dHz", ODRoptionLower);
			}
			else
			{
				configArray.push_back(XsOutputConfiguration(XDI_Temperature, ODRoption));
				ROS_INFO("XDI_Temperature, %dHz", ODRoption);
			}
		}

		bool enableHRData = false;
		if (ros::param::get("~enable_high_rate", enableHRData) && enableHRData)
		{
			int ODRHRGyroOption = 100;
			int ODRHRAccOption = 100;
			//TODO: to make it more dummy, you should check the different option value to the allowed data rates for different models...
			if(ros::param::get("~output_data_rate_acchr", ODRHRAccOption) && ros::param::get("~output_date_rate_gyrohr", ODRHRGyroOption))
			{
				configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, ODRHRGyroOption));
				ROS_INFO("XDI_RateOfTurnHR, %dHz", ODRHRGyroOption);
				configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, ODRHRAccOption));
				ROS_INFO("XDI_AccelerationHR, %dHz", ODRHRAccOption);
			}
			else{
				ROS_ERROR("Couldn't find output_data_rate_acchr and output_date_rate_gyrohr parameters, won't configure the output.");
			}

		}
		

		if(isDeviceVruAhrs || isDeviceGnssIns)
		{
			if(ros::param::get("~pub_quaternion", should_config) && should_config)
			{
				if(isMTiX)
				{
					configArray.push_back(XsOutputConfiguration(XDI_Quaternion, ODRoptionLower));
					ROS_INFO("XDI_Quaternion, %dHz", ODRoptionLower);
				}
				else
				{
					configArray.push_back(XsOutputConfiguration(XDI_Quaternion, ODRoption));
					ROS_INFO("XDI_Quaternion, %dHz", ODRoption);
				}
			}
			if(ros::param::get("~pub_free_acceleration", should_config) && should_config)
			{
				if(isMTiX)
				{
					configArray.push_back(XsOutputConfiguration(XDI_FreeAcceleration, ODRoptionLower));
					ROS_INFO("XDI_FreeAcceleration, %dHz", ODRoptionLower);
				}
				else
				{
					configArray.push_back(XsOutputConfiguration(XDI_FreeAcceleration, ODRoption));
					ROS_INFO("XDI_FreeAcceleration, %dHz", ODRoption);
				}
			}
		}

		if(isDeviceGnssIns)
		{
			if(ros::param::get("~pub_positionLLA", should_config) && should_config)
			{
				if(isMTiX)
				{
					configArray.push_back(XsOutputConfiguration(XDI_LatLon | XDI_SubFormatFp1632, ODRoptionLower)); //FP1632
					configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid | XDI_SubFormatFp1632, ODRoptionLower)); //FP1632
					ROS_INFO("XDI_LatLon, XDI_AltitudeEllipsoid, %dHz", ODRoptionLower);
				}
				else
				{
					configArray.push_back(XsOutputConfiguration(XDI_LatLon | XDI_SubFormatFp1632, ODRoption)); //FP1632
					configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid | XDI_SubFormatFp1632, ODRoption)); //FP1632
					ROS_INFO("XDI_LatLon, XDI_AltitudeEllipsoid, %dHz", ODRoption);
				}
			}
			//pub_velocity
			if(ros::param::get("~pub_velocity", should_config) && should_config)
			{
				if(isMTiX)
				{
					configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ | XDI_SubFormatFp1632, ODRoptionLower)); //FP1632
					ROS_INFO("XDI_VelocityXYZ, %dHz", ODRoptionLower);
				}
				else
				{
					configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ | XDI_SubFormatFp1632, ODRoption)); //FP1632
					ROS_INFO("XDI_VelocityXYZ, %dHz", ODRoption);
				}
			}

			//baro for mti-7/8 max 50Hz, for mti-670/680 max 100Hz.
			if (ros::param::get("~pub_pressure", should_config) && should_config)
			{	
				//if it is mtix, max is 50(output_data_rate_baro_mtione), if not, max is 100(use ODRoptionLower).
				if(isMTiX)
				{
					//ros::param read the output_data_rate_baro_mtione
					int ODRoptionBaro = 50;
					if(ros::param::get("~output_data_rate_baro_mtione", ODRoptionBaro))
					{
						if(ODRoptionBaro <= 50)
						{
							configArray.push_back(XsOutputConfiguration(XDI_BaroPressure, ODRoptionBaro));
							ROS_INFO("XDI_BaroPressure, 50Hz");
						}
						else
						{
							configArray.push_back(XsOutputConfiguration(XDI_BaroPressure, 50));
							ROS_INFO("XDI_BaroPressure default at 50Hz");
						}
					}
					else
					{
						configArray.push_back(XsOutputConfiguration(XDI_BaroPressure, 50));
						ROS_INFO("XDI_BaroPressure default at 50Hz");
					}
				}
				else
				{
					configArray.push_back(XsOutputConfiguration(XDI_BaroPressure, ODRoptionLower));
					ROS_INFO("XDI_BaroPressure, %dHz", ODRoptionLower);
				}
			}

			//XDI_GnssPvtData output data rate is fixed at 4Hz.
			configArray.push_back(XsOutputConfiguration(XDI_GnssPvtData, 4));
			ROS_INFO("XDI_GnssPvtData, 4Hz");

		}

		configArray.push_back(XsOutputConfiguration(XDI_StatusWord, 0));
		ROS_INFO("XDI_StatusWord");

		if (!m_device->setOutputConfiguration(configArray))
			return handleError("Could not configure MTi device. Aborting.");

		ROS_INFO("Sensor output configured successfully.");


		
		//ROS_INFO print the 5th to 7th characters of the product code to check if it is mti-680(G) or other mti-600 models.
		bool isMTi620 = false;
		bool isMTi630 = false;
		bool isMTi670 = false;
		bool isMTi680 = false;
		bool isMTiG710 = false;
		isMTi620 = m_productCode.toStdString().substr(4, 3) == "620";
		isMTi630 = m_productCode.toStdString().substr(4, 3) == "630";
		isMTi670 = m_productCode.toStdString().substr(4, 3) == "670";
		isMTi680 = m_productCode.toStdString().substr(4, 3) == "680";
		isMTiG710 = m_device->deviceId().isMtig();

		ROS_INFO("Configuring Option Flags.....");
		//TODO: check if MTi-100 has this feature or not...
		if(!m_device->deviceId().isImu())
		{
			//enable_inrun_compass_calibration
			bool enable_inrun_compass_calibration = false;
			if(ros::param::get("~enable_inrun_compass_calibration", enable_inrun_compass_calibration))
			{
				if(enable_inrun_compass_calibration)
				{
					
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnableInrunCompassCalibration, XsDeviceOptionFlag::XDOF_None))
					{
						ROS_INFO("Enable In-run Compass Calibration Success!");
					}
					else
					{
						ROS_WARN("Enable In-run Compass Calibration Failed!");
					}
				}
				else
				{
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnableInrunCompassCalibration))
					{
						ROS_INFO("Disable In-run Compass Calibration Success!");
					}
					else
					{
						ROS_WARN("Disable In-run Compass Calibration Failed!");
					}
				}

			}

			bool enable_rotsensor_frame_config = false;
			if(ros::param::get("~enable_rotsensor_frame_config", enable_rotsensor_frame_config) && enable_rotsensor_frame_config)
			{
				std::vector<double> rotsensor_rotation_euler = {0.0, 0.0, 0.0};
				if(ros::param::get("~rotsensor_rotation_euler", rotsensor_rotation_euler))
				{
					
					//change sensor's RotSensor frame by euler angles, roll, pitch, yaw
					XsEuler rotsensor_euler(rotsensor_rotation_euler[0], 
						rotsensor_rotation_euler[1], 
						rotsensor_rotation_euler[2]);
					ROS_INFO("Setting rotsensor alignment rotation from Euler angles (roll: %.2fdeg, pitch: %.2fdeg, yaw: %.2fdeg)...", rotsensor_euler.roll(), rotsensor_euler.pitch(), rotsensor_euler.yaw());

					// Convert Euler angles to quaternion
					XsQuaternion rotsensor_quat(rotsensor_euler);

					// Normalize the quaternion
					rotsensor_quat.normalize();

					// Apply the alignment rotation quaternion to the device
					if (!m_device->setAlignmentRotationQuaternion(XAF_Sensor, rotsensor_quat)) {
						ROS_WARN("Failed to set alignment rotation quaternion");
						return false;
					} else {
						ROS_INFO("Successfully set alignment rotation from Euler angles");
						return true;
					}

				}

				
			}

		}
		//AHS is for MTI-2/3/320, MTI-200/300, but for MTI-620/630, it is on the filter profile VRUAHS.
		//not imu, not gnss, not 600
		if(!m_device->deviceId().isImu() && !m_device->deviceId().isGnss() && !m_device->deviceId().isMti6X0())
		{
			//enable_active_heading_stabilization
			bool enable_active_heading_stabilization = false;
			if(ros::param::get("~enable_active_heading_stabilization", enable_active_heading_stabilization))
			{
				if(enable_active_heading_stabilization)
				{
					
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnableInrunCompassCalibration, XsDeviceOptionFlag::XDOF_None))
					{
						ROS_INFO("Enable Active Heading Stabilization Success!");
					}
					else
					{
						ROS_WARN("Enable Active Heading Stabilization Failed!");
					}
				}
				else
				{
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnableInrunCompassCalibration))
					{
						ROS_INFO("Disable Active Heading Stabilization Success!");
					}
					else
					{
						ROS_WARN("Disable Active Heading Stabilization Failed!");
					}
				}

			}

		}


		


		if (isMTi670 || isMTi680 || isMTiG710)
		{
			// use ros:param to get 
			//enable_orientation_smoother, enable_position_velocity_smoother, enable_continuous_zero_rotation_update
			bool enable_orientation_smoother = false;
			if(ros::param::get("~enable_orientation_smoother", enable_orientation_smoother))
			{
				if(enable_orientation_smoother)
				{
					
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnableOrientationSmoother, XsDeviceOptionFlag::XDOF_None))
					{
						ROS_INFO("Enable Orientation Smoother Success!");
					}
					else
					{
						ROS_WARN("Enable Orientation Smoother Failed!");
					}
				}
				else
				{
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnableOrientationSmoother))
					{
						ROS_INFO("Disable Orientation Smoother Success!");
					}
					else
					{
						ROS_WARN("Disable Orientation Smoother Failed!");
					}
				}

			}


			//position velocity smoother, and continous zero rotation update is for MTi-680(G) only.
			if (isMTi680)
			{
				bool enable_position_velocity_smoother = false;
				if(ros::param::get("~enable_position_velocity_smoother", enable_position_velocity_smoother))
				{
					if(enable_position_velocity_smoother)
					{
						
						if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnablePositionVelocitySmoother, XsDeviceOptionFlag::XDOF_None))
						{
							ROS_INFO("Enable Position Velocity Smoother Success!");
						}
						else
						{
							ROS_WARN("Enable Position Velocity Smoother Failed!");
						}
					}
					else
					{
						if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnablePositionVelocitySmoother))
						{
							ROS_INFO("Disable Position Velocity Smoother Success!");
						}
						else
						{
							ROS_WARN("Disable Position Velocity Smoother Failed!");
						}
					}

				}

				bool enable_continuous_zero_rotation_update = false;
				if(ros::param::get("~enable_continuous_zero_rotation_update", enable_continuous_zero_rotation_update))
				{
					if(enable_continuous_zero_rotation_update)
					{
						
						if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnableContinuousZRU, XsDeviceOptionFlag::XDOF_None))
						{
							ROS_INFO("Enable Continous Zero Rotation Update Success!");
						}
						else
						{
							ROS_WARN("Enable Continous Zero Rotation Update Failed!");
						}
					}
					else
					{
						if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnableContinuousZRU))
						{
							ROS_INFO("Disable Continous Zero Rotation Update Success!");
						}
						else
						{
							ROS_WARN("Disable Continous Zero Rotation Update Failed!");
						}
					}

				}
			}
			

		}

		ROS_INFO("Configuring GNSS relevant Prameters...");

		// read yaml config for gnss lever arm, only for MTI-8 or MTI-680(G):
		if (m_device->deviceId().isRtk())
		{
			if (ros::param::has("~GNSS_LeverArm"))
			{
				std::vector<double> gnssLeverArm;
				ros::param::get("~GNSS_LeverArm", gnssLeverArm);
				if (gnssLeverArm.size() != 3)
					return handleError("Invalid GNSS_LeverArm parameter");
				XsVector3 leverArm;
				leverArm[0] = gnssLeverArm[0];
				leverArm[1] = gnssLeverArm[1];
				leverArm[2] = gnssLeverArm[2];
				ROS_INFO("Setting GNSS lever arm to: %f, %f, %f", leverArm[0], leverArm[1], leverArm[2]);
				if (!m_device->setGnssLeverArm(leverArm))
					return handleError("Could not set GNSS lever arm");
			}
			else
			{
				ROS_WARN("No GNSS_LeverArm parameter found, using default value");
			}
		}

		if (m_device->deviceId().isGnss())
		{
			// Set the GNSS platform
			XsUbloxGnssPlatform platform = XGP_Portable;
			if (ros::param::has("~ublox_platform"))
			{
				int platformParam = 0;
				ros::param::get("~ublox_platform", platformParam);
				platform = (XsUbloxGnssPlatform)platformParam;
				ROS_INFO("Setting GNSS platform to: %d", platform);
				if (!m_device->setUbloxGnssPlatform(platform))
					return handleError("Could not set GNSS platform");
			}
			else
			{
				ROS_WARN("No ublox_platform parameter found, using default value");
			}

			bool enable_beidou = false;
			if(ros::param::get("~enable_beidou", enable_beidou))
			{
				if(enable_beidou)
				{
					
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_EnableBeidou, XsDeviceOptionFlag::XDOF_None))
					{
						ROS_INFO("Enable Beidou Success!");
					}
					else
					{
						ROS_WARN("Enable Beidou Failed!");
					}
				}
				else
				{
					if(m_device->setDeviceOptionFlags(XsDeviceOptionFlag::XDOF_None, XsDeviceOptionFlag::XDOF_EnableBeidou))
					{
						ROS_INFO("Disable Beidou Success!");
					}
					else
					{
						ROS_WARN("Disable Beidou Failed!");
					}
				}

			}
			
		}

		
		//set filter profiles
		bool enable_filter_config = false;
		if (ros::param::get("~enable_filter_config", enable_filter_config) && enable_filter_config)
		{
			XsFilterProfileArray onboardProfiles = m_device->availableOnboardFilterProfiles();
			if(!onboardProfiles.empty())
			{
				int filterIndex = 0; // Initialize counter variable
				std::map<int, std::string> profileDictionary; 
				ROS_INFO("Got Onboard Filter Profiles: ");
				for (XsFilterProfileArray::iterator it = onboardProfiles.begin(); it != onboardProfiles.end(); ++it, ++filterIndex)
				{
					std::ostringstream msg;
					/// Cast type() and version() to unsigned int before appending to ensure they are treated as numbers
					msg << static_cast<unsigned int>((*it).type()) << "." << static_cast<unsigned int>((*it).version()) << " " << (*it).label();
					profileDictionary[filterIndex] = msg.str();
					ROS_INFO("Index %d = %s", filterIndex, msg.str().c_str());
				}

				int filterIndexToSet = 0;
				std::string rollpitchLabel = "Robust";
				std::string yawLabel = "VRUAHS";
				bool isGotFilterParam = false;
				if(isMTi620 || isMTi630)
				{
					if(ros::param::get("~mti620630filterlabel_rollpitch", rollpitchLabel) && ros::param::get("~mti620630filterlabel_yaw", yawLabel))
					{
						isGotFilterParam = true;
					}
				}
				else
				{
					if(ros::param::get("~mti_filter_option", filterIndexToSet))
					{
						if(filterIndexToSet < onboardProfiles.size())
						{
							isGotFilterParam = true;
						}
						else
						{
							ROS_WARN("Your mti_filter_option %d is incorrect, it is bigger than index bound of 0 to %d, please check your model and correct the value at xsens_mti_node.yaml.", filterIndexToSet, (int)(onboardProfiles.size() - 1));
						}
						
					}

				}
			

				if(isGotFilterParam)
				{
					std::string profileContent = profileDictionary[filterIndexToSet];
					if(m_device->deviceId().isMti6X0())
					{
						if(isMTi670 || isMTi680)
						{
							//for MTI-670/680 series, use label()
							if(m_device->setOnboardFilterProfile(onboardProfiles[filterIndexToSet].label()))
							{
								ROS_INFO("Successfully Set Onboard Filter Option to Index: %d, Name: %s", filterIndexToSet, profileContent.c_str());
							}
							else
							{
								ROS_WARN("Failed to Set Onboard Filter Option to Index: %d, Name: %s", filterIndexToSet, profileContent.c_str());
							}
						}
						else
						{
							//For MTI-620 or MTI-630, for example: "Responsive/VRU"
							XsString filterToSet = XsString(rollpitchLabel) + XsString("/") + XsString(yawLabel);
							
								if(m_device->setOnboardFilterProfile(filterToSet))
								{
									ROS_INFO("Successfully Set Onboard Filter Option to Name: %s", filterToSet.c_str());
								}
								else
								{
									ROS_WARN("Failed to Set Onboard Filter Option to Name: %s", filterToSet.c_str());
								}
						}

					}
					else
					{
						//for MTI-1/10/100 series, use type()
						if(m_device->setOnboardFilterProfile(onboardProfiles[filterIndexToSet].type()))
						{
							ROS_INFO("Successfully Set Onboard Filter Option to Index: %d, Name: %s", filterIndexToSet, profileContent.c_str());
						}
						else
						{
							ROS_WARN("Failed to Set Onboard Filter Option to Index: %d, Name: %s", filterIndexToSet, profileContent.c_str());
						}
					}
				}

			}
			

		}
		


		//Lastly, set the baudrate.
		bool enableSettingBaudrate = false;
		if (ros::param::get("~enable_setting_baudrate", enableSettingBaudrate) && enableSettingBaudrate)
		{
			int setBaudrateParam = 0;
			ros::param::get("~set_baudrate_value", setBaudrateParam);
			ROS_INFO("Found baudrate parameter to set: %d", setBaudrateParam);
			XsBaudRate baudrate = XBR_115k2;
			baudrate = XsBaud::numericToRate(setBaudrateParam);

			//set baudrate to 921600 if the above data output is too much, 400Hz.
			if (enableHRData)
			{
				baudrate = XBR_2000k;
				if (!m_device->setSerialBaudRate(baudrate))
					return handleError("Could not set baudrate to " + std::to_string(XsBaud::rateToNumeric(baudrate)));
				
				ROS_INFO("Since the HighRate Data is enabled, Sensor baudrate forcely configured to %d.", XsBaud::rateToNumeric(baudrate));
			}
			else
			{
				if(ODRoption >= 400 && baudrate < XBR_921k6)
				{
					baudrate = XBR_921k6;
					if (!m_device->setSerialBaudRate(baudrate))
						return handleError("Could not set baudrate to " + std::to_string(XsBaud::rateToNumeric(baudrate)));

					ROS_INFO("Since 400Hz data is enabled, Sensor baudrate forcely configured to %d.", XsBaud::rateToNumeric(baudrate));
				}
				else
				{
					if (!m_device->setSerialBaudRate(baudrate))
						return handleError("Could not set baudrate to " + std::to_string(XsBaud::rateToNumeric(baudrate)));

					ROS_INFO("Sensor baudrate configured to %d success.", XsBaud::rateToNumeric(baudrate));
				}		
			}

		}

		//now we sleep a little while, since the configuration commands might take a little time.
		ros::Duration(0.05).sleep();

	}



	return true;



}
