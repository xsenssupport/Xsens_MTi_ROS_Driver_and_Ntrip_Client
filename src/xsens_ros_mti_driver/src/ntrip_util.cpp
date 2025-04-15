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

#include "ntrip_util.h"

#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <fstream>
#include <memory>

// #include <ros/ros.h>

namespace libntrip
{

  namespace
  {
    double DegreeConvertToDDMM(double const &degree)
    {
      // Get the absolute value of the degree
      double abs_degree = fabs(degree);
      
      // Extract the whole degree part
      int deg = static_cast<int>(floor(abs_degree));
      
      // Calculate minutes from the fractional part of the degree
      double minute = (abs_degree - deg) * 60.0;
      
      // Return in DDMM.MMMMM format (degree * 100 + minutes)
      return (deg * 100.0 + minute);
    }

    uint8_t DetermineFixType(uint32_t status, const XsRawGnssPvtData& gnssPvtData)
    {
      // Check RTK status first (from StatusWord bits 27-28)
      uint8_t rtk_status = (status >> 27) & 0x3; // status & 0x18000000;
      if (rtk_status == 2)
      {
        return 0x04; // 4 = RTK fixed
      }
      else if (rtk_status == 1)
      {
        return 0x05; // 5 = RTK float
      }
      
      // If no RTK, check GNSS fix bit from StatusWord
      bool gnssFix = status & (1 << 2);
      
      // If GNSS fix, determine the type based on GnssPvtData
      if (gnssFix)
      {
        // Check GnssPvtData fix type
        uint8_t gnssPvtFixType = gnssPvtData.m_fixType;
        
        switch (gnssPvtFixType)
        {
          case 0x00: // No Fix
            return 0x00;
          case 0x01: // Dead Reckoning only
            return 0x06; // 6 = Estimated/Dead reckoning fix
          case 0x02: // 2D-Fix
          case 0x03: // 3D-Fix
          case 0x04: // GNSS + dead reckoning combined
            return 0x01; // 1 = Autonomous GNSS fix
          case 0x05: // Time only fix
            return 0x00; // No position fix
          default:
            return 0x00;
        }
      }
      
      return 0x00; // Default to no fix
    }

  }// namespace

  //
  // Ntrip util.
  //

  int BccCheckSumCompareForGGA(const char *src) {
    int sum = 0;
    int num = 0;
    sscanf(src, "%*[^*]*%x", &num);
    for (int i = 1; src[i] != '*'; ++i) {
      sum ^= src[i];
    }
    return sum - num;
  }


  /*
  It is better to select pvtData and statusword from MT Manager - Device Settings - Output Configurations
  1- if packet has pvtData and statusword, then parse the pvt data and statusword to GPGGA
  2- if the packet has utc time, statusword, lat, long, then combine the data to GPGGA, but it doesn't have the m_numSv/m_hdop/m_hMsl
  2- else, create an empty GPGGA.
  ref for GPGGA, <<u-blox ZED-F9P Interface Description>> UBX-18010854 - R07,  10 July 2019, page 15:
  https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf
  $xxGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,altUnit,sep,sepUnit,diffAge,diffStation*cs<CR><LF>
  xxGGA: string
  time: hhmmss.ss
  lat:ddmm.mmmmm
  NS: character, N or S
  lon: dddmm.mmmmm
  EW: character, E or W
  quality: 0 = No fix, 1 = Autonomous GNSS fix, 2 = Differential GNSS fix, 4 = RTK fixed, 5 = RTK float, 6 = Estimated/Dead reckoning fix
  numSV : Number of satellites used (range: 0-12)
  HDOP: Horizontal Dilution of Precision (range: 0.5-99.9)
  alt: Altitude above mean sea level in meter
  altUnit: character, M(meters, fixed field)
  sep: numeric, meter, Geoid separation: difference between ellipsoid and mean sea level
  sepUnit: character, M(meters, fixed field)
  diffAge: numeric, Age of differential corrections (null when DGPS is not used)
  diffStation: numeric, ID of station providing differential corrections (null when DGPS is not used)
  cs: hexadecimal, Checksum, example: *5B
  <CR><LF>: character, Carriage return and Line feed
  */
 int generateGGA(const XsDataPacket &packet, std::string *gga_out)
  {
    if (gga_out == nullptr)
      return -1;

    //both pvtData and statusword are required to generate GPGGA
    if (!packet.containsStatus() || !packet.containsRawGnssPvtData())
    {
      return -1;
    }

    char src[256] = {0};
    char *ptr = src;

    uint8_t fixType = 0x00;
    int hour = 0, min = 0;
    // The `99.99` for HDOP indicates that the horizontal dilution of precision is very high, or the accuracy of the horizontal position data is very poor or unreliable.
    double sec = 0.0, lat = 0.0, lon = 0.0, hdop = 0.0, alt = 0.0, geoidSep = 0.0;
    int numSv = 0;

    uint32_t status = 0;
    XsRawGnssPvtData gnssPvtData;

    if (packet.containsStatus())
    {
      status = packet.status();
    }

    if (packet.containsRawGnssPvtData())
    {
      gnssPvtData = packet.rawGnssPvtData();
      hour = gnssPvtData.m_hour;
      min = gnssPvtData.m_min;
      sec = gnssPvtData.m_sec + gnssPvtData.m_nano * 1e-9;
      lat = gnssPvtData.m_lat;
      lon = gnssPvtData.m_lon;
      numSv = gnssPvtData.m_numSv;
      hdop = static_cast<double>(gnssPvtData.m_hdop) / 100.0;
      alt = static_cast<double>(gnssPvtData.m_hMsl) / 1000.0; // cast to double and convert to meters
      geoidSep = static_cast<double>(gnssPvtData.m_height - gnssPvtData.m_hMsl) / 1000.0;
    }

    // Determine fix type using both status and GnssPvtData
    fixType = DetermineFixType(status, gnssPvtData);

    // Ensure that the latitude and longitude are converted and formatted properly
    char latDir = lat >= 0.0 ? 'N' : 'S';
    double latDDMM = fabs(DegreeConvertToDDMM(lat* 1e-7));

    char lonDir = lon >= 0.0 ? 'E' : 'W';
    double lonDDMM = fabs(DegreeConvertToDDMM(lon* 1e-7));
    // Debug only, Print out the variables using ROS_INFO
    //ROS_INFO("hour: %02d, min: %02d, sec: %05.2f, latDDMM: %012.5f, latDir: %c, lonDDMM: %013.5f, lonDir: %c, fixType: %01d, numSv: %02d, hdop: %.1f, alt: %.2f",
    //  hour, min, sec, latDDMM, latDir, lonDDMM, lonDir, fixType, numSv, hdop, alt);


    //example: "$GPGGA,162220.00,3123.99529,N,12149.95179,E,2,12,0.6,38.87,M,0.000,M,,0000*5F\r\n"


    // Use snprintf with the correct remaining buffer size and proper variable usage
    // For higher precision of lat long, change to %012.7f,%013.7f for lat and lon
    ptr += snprintf(ptr, sizeof(src)+src-ptr,
                    "$GPGGA,%02d%02d%05.2f,%010.5f,%c,%011.5f,%c,%01d,"
                    "%02d,%.1f,%.2f,M,%.2f,M,0,0",
                    hour, min, sec,
                    latDDMM, latDir,
                    lonDDMM, lonDir,
                    fixType,
                    numSv,
                    hdop,
                    alt,
                    geoidSep); 

    uint8_t checksum = 0;
    for (char *q = src + 1; q <= ptr; q++) {
      checksum ^= *q; // check sum.
    }
    ptr += snprintf(ptr, sizeof(src)+src-ptr, "*%02X%c%c", checksum, 0x0D, 0x0A);
    *gga_out = std::string(src, ptr-src);
    
    return BccCheckSumCompareForGGA(gga_out->c_str());
  }

} // namespace libntrip
