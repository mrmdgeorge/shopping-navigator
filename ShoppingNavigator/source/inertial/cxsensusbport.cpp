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
 * @file   cxsensusbport.cpp
 * @brief  A simple class to wrap Xsens IMU USB connectivity
 * @author M. George
 */

#include "basil/basil.h"

#include "cxsensusbport.h"

/**************************************************************************//**
 * Constructor for cXsensUsbPort
 * \return None
 ******************************************************************************/
cXsensUsbPort::cXsensUsbPort() : mPacket((unsigned short)1,false)
{
    // Request cal data (acc,gyro,mag.) and euler data
    mOutputMode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
    mSettings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;


    // Set constant components of the basil header we will add to it
    mData.header.sync[0] = 'C';
    mData.header.sync[1] = 'M';
    mData.header.sync[2] = 'U';
    mData.header.headerSize = sizeof(BASIL::BasilHeader);
    mData.header.hardwareId = 0;
    mData.header.deviceId = 0;
    mData.header.messageType = static_cast<uint8_t>(BASIL::MESSAGETYPE::IMU);
    mData.header.sensorModel = static_cast<uint8_t>(BASIL::IMUS::XSENS_MTI);
    mData.header.dataSize = sizeof(BASIL::XsensData) - sizeof(BASIL::BasilHeader) - sizeof(BASIL::BasilCrc);
    mData.header.reserved = 0;
    mData.basilCrc = 0;
}

/**************************************************************************//**
 * Destructor for cXsensUsbPort
 * \return None
 ******************************************************************************/
cXsensUsbPort::~cXsensUsbPort()
{
    disconnect();
}

//! Do any setup necessary for a connection here
bool cXsensUsbPort::configure()
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
void cXsensUsbPort::getPacket(char* buffer)
{
    // Get data in Xsens formats
    mImu.waitForDataMessage(&mPacket);
    mRawData = mPacket.getCalData(0);
    mEulerData = mPacket.getOriEuler(0);

    // Set the time
    mData.header.time.setTime(BASIL::TIME::FREE_RUNNING,0,static_cast<double>(mPacket.getRtc())*1e-3);

    // Set the non-constant elements of the BASIL IMU packet
    mData.acceleration[0] = mRawData.m_acc.m_data[0];
    mData.acceleration[1] = mRawData.m_acc.m_data[1];
    mData.acceleration[2] = mRawData.m_acc.m_data[2];
    mData.angularRate[0] = mRawData.m_gyr.m_data[0];
    mData.angularRate[1] = mRawData.m_gyr.m_data[1];
    mData.angularRate[2] = mRawData.m_gyr.m_data[2];
    mData.magnetometer[0] = mData.magnetometer[1] = mData.magnetometer[2] = 0.0;
    mData.messageId = 0;
    mData.messageLength = 0;
    mData.checksum = 0;

    // Copy to the buffer
    memcpy(buffer, &mData, sizeof(BASIL::XsensData));
}

/**************************************************************************//**
 * Diconnect here
 * \return None
 ******************************************************************************/
void cXsensUsbPort::disconnect()
{
    mImu.closePort();
}

int cXsensUsbPort::scanHardware()
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
