/**********************************************************************************
 *  The MIT License (MIT)                                                        *
 *                                                                                *
 *  Copyright (c) 2014 Carnegie Mellon University                                 *
 *                                                                                *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy  *
 *  of this software and associated documentation files (the "Software"), to deal *
 *  in the Software without restriction, including without limitation the rights  *
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 *  copies of the Software, and to permit persons to whom the Software is         *
 *  furnished to do so, subject to the following conditions:                      *
 *                                                                                *
 *  The above copyright notice and this permission notice shall be included in    *
 *  all copies or substantial portions of the Software.                           *
 *                                                                                *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
 *  THE SOFTWARE.                                                                 *
 **********************************************************************************/
/**
 * @file   cxsensusbport.h
 * @brief  A simple class to wrap Xsens IMU USB connectivity
 * @author M. George
 */

#ifndef CXSENSUSBPORT_H
#define CXSENSUSBPORT_H

#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>

#include "basil/messages/sensors/simumsg.h"

#include "xsens-mti/cmtdef.h"
#include "xsens-mti/xsens_time.h"
#include "xsens-mti/xsens_list.h"
#include "xsens-mti/cmtscan.h"
#include "xsens-mti/cmt3.h"

using namespace xsens;

template<typename Interface>
class cXsensUsbPort
{
public:
    cXsensUsbPort();

    ~cXsensUsbPort();

    //! Extract a self contained BASIL formatted data packet from the log
    int getPacket(char*);

    //! Do any setup necessary for a connection here
    bool configure();

    //! Connect to the IMU permanently
    void connect();

    //! End connection
    void disconnect();

    //! A member copy of the interface which can be stateful
    Interface mInterface;

private:
    //! Is the socket configured
    bool mConfigured;

    //! Static buffer for incoming data
    char mBuffer[MAX_MESSAGE_LENGTH];

    //! Number of bytes read
    int mBytesReceived;

    xsens::Cmt3 mImu;
    unsigned long mImuCount;
    unsigned int mSampleFrequency;
    CmtDeviceId mDeviceId;
    List<CmtPortInfo> mPortInfo;

    // Raw data
    CmtCalData mRawData;
    // Filtered euler angle data
    CmtEuler mEulerData;
    Packet mPacket;

    BASIL::IMU::XSENS::MTi::Msg mImuMsg;

    CmtOutputMode mOutputMode;
    CmtOutputSettings mSettings;

    XsensResultValue mResult;

    //! Scan for an XSens device
    int scanHardware();
};

template<typename Interface>
cXsensUsbPort<Interface>::cXsensUsbPort() : mPacket(static_cast<unsigned short>(1),false)
{
    // Request cal data (acc,gyro,mag.) and euler data
    mOutputMode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
    mSettings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

    // Set constant components of the basil header we will add to it
    mImuMsg.header.sync[0] = 'C';
    mImuMsg.header.sync[1] = 'M';
    mImuMsg.header.sync[2] = 'U';
    mImuMsg.header.headerSize = sizeof(BASIL::BasilHeader);
    mImuMsg.header.hardwareId = 0;
    mImuMsg.header.deviceId = 0;
    mImuMsg.header.messageType = static_cast<uint8_t>(BASIL::MESSAGE::IMU);
    mImuMsg.header.messageVersion = static_cast<uint8_t>(BASIL::IMU::XSENS_MTI);
    mImuMsg.header.dataSize = sizeof(IMU::XSENS::MTi::Msg) - sizeof(BASIL::BasilHeader) - sizeof(BASIL::BasilCrc);
    mImuMsg.header.reserved = 0;
    mImuMsg.crc = 0;

    mImuMsg.payload.preamble = 0xFA;
    mImuMsg.payload.busid = 0xFF;
}

template<typename Interface>
cXsensUsbPort<Interface>::~cXsensUsbPort()
{
    disconnect();
}

template<typename Interface>
void cXsensUsbPort<Interface>::disconnect()
{
    mImu.closePort();
}

//! Do any setup necessary for a connection here
template<typename Interface>
bool cXsensUsbPort<Interface>::configure()
{
    // Perform hardware scan
    int mtCount = scanHardware();
    std::cout << mtCount << " Xsens Devices Found" << std::endl;

    // Check for successful detection
    if (mtCount == 0)
    {
        std::cout << "No XSens devices found" << std::endl;
        mImu.closePort();
        return false;
    }

    // Set device to user input settings
    mResult = mImu.gotoConfig();
    if (mResult != XRV_OK)
    {
        std::cout << "Error: XSens device could not open config." << std::endl;
        return false;
    }

    mSampleFrequency = mImu.getSampleFrequency();

    // set the device output mode for the device(s)
    CmtDeviceMode deviceMode(mOutputMode, mSettings, mSampleFrequency);
    mResult = mImu.setDeviceMode(deviceMode,true,mDeviceId);
    if (mResult != XRV_OK)
    {
        std::cout << "Error: XSens device could not set device mode." << std::endl;
        return false;
    }

    // start receiving data
    mResult = mImu.gotoMeasurement();
    if (mResult != XRV_OK)
    {
        std::cout << "Error: XSens device could not return to measurement." << std::endl;
        return false;
    }

    return true;
}

//! Extract a self contained BASIL data packet
template<typename Interface>
int cXsensUsbPort<Interface>::getPacket(char* buffer)
{
    // Get data in Xsens formats
    mImu.waitForDataMessage(&mPacket);
    mRawData = mPacket.getCalData(0);
    mEulerData = mPacket.getOriEuler(0);

    // Set the time
    mImuMsg.header.time.setTime(BASIL::TIME::FREE_RUNNING,0,static_cast<double>(mPacket.getRtc())*1e-3);

    // Set the non-constant elements of the BASIL IMU packet
    mImuMsg.payload.acceleration[0] = mRawData.m_acc.m_data[0];
    mImuMsg.payload.acceleration[1] = mRawData.m_acc.m_data[1];
    mImuMsg.payload.acceleration[2] = mRawData.m_acc.m_data[2];
    mImuMsg.payload.angularRate[0] = mRawData.m_gyr.m_data[0];
    mImuMsg.payload.angularRate[1] = mRawData.m_gyr.m_data[1];
    mImuMsg.payload.angularRate[2] = mRawData.m_gyr.m_data[2];
    mImuMsg.payload.magnetometer[0] = mImuMsg.payload.magnetometer[1] = mImuMsg.payload.magnetometer[2] = 0.0;
    mImuMsg.payload.messageId = 0;
    mImuMsg.payload.messageLength = 0;
    mImuMsg.payload.checksum = 0;

    // Copy to the buffer
    memcpy(buffer, &mImuMsg, sizeof(IMU::XSENS::MTi::Msg));
    return sizeof(IMU::XSENS::MTi::Msg);
}

// Connect to the port permanently.  This is the main entry point for a process or thread
template<typename Interface>
void cXsensUsbPort<Interface>::connect()
{
    // Configure the connection
    mConfigured = configure();

    // Wait on data permanently
    while (mConfigured)
    {
        // Get individual datagram
        mBytesReceived = getPacket(mBuffer);

        // Send it to the interface for processing
        if (mBytesReceived > 0)
        {
            mInterface.parse(mBuffer, static_cast<unsigned int>(mBytesReceived));
        }
    }
}

template<typename Interface>
int cXsensUsbPort<Interface>::scanHardware()
{
    unsigned long portCount = 0;
    int mtCount;

    std::cout << "Scanning for connected Xsens devices..." << std::endl;
    xsens::cmtScanPorts(mPortInfo);
    portCount = mPortInfo.length();
    std::cout << "Done scanning." << std::endl;;

    if (portCount == 0)
    {
        std::cout << "No MotionTrackers found" << std::endl;
        return 0;
    }

    std::cout << "Using COM port " << mPortInfo[0].m_portName << ", at " << mPortInfo[0].m_baudrate << std::endl;

    std::cout << "Opening port..." << std::endl;
    mResult = mImu.openPort(mPortInfo[0].m_portName, mPortInfo[0].m_baudrate);
    if (mResult != XRV_OK)
    {
        std::cout << "Error: XSens device could not open port." << std::endl;
        return false;
    }
    std::cout << "Done opening." << std::endl;

    // retrieve the device IDs
    std::cout << "Retrieving MotionTracker device ID" << std::endl;
    mResult = mImu.getDeviceId((unsigned char)(1), mDeviceId);
    if (mResult != XRV_OK)
    {
        std::cout << "Error: XSens device given ID: " << mDeviceId << std::endl;
        return false;
    }

    return true;
}



#endif // CXSENSUSBPORT_H
