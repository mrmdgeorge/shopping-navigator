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
 * @file   cstepdetector.h
 * @brief  A step detection class which recognizes step signatures in IMU data
 * @author M. George
 */

#ifndef CSTEPDETECTOR_H
#define CSTEPDETECTOR_H

#include <deque>
#include <cmath>

#include <boost/property_tree/ptree.hpp>

/**************************************************************************//**
* \class cStepDetector
* \brief Class for detecting steps from accelerometer data.  
* Returns true after impact spike settles.
******************************************************************************/
template<unsigned int FilterLength>
class cStepDetector
{
public:
    //! Constructor
    cStepDetector(boost::property_tree::ptree config);

    //! Destructor
    ~cStepDetector(){}

    //! Interface for detecting zupts from an IMU's accelerometers
    bool detect(double acceleration);

private:
    //! Storage queue for vertical accelerations and low-pass filtered version
    std::deque<double> mAccelerationBuffer;
    std::deque<double> mFilterBuffer;

    //! How many steps have been detected
    unsigned long mStepCount;

    //! Count how many IMU messages received from hardware.
    unsigned long mMessageCount;
    
    //! Low pass Butterworth filter dimension
    static const unsigned int mFilterLength = FilterLength;
    
    //! Low pass Butterworth filter coefficients
    Eigen::Matrix<double,FilterLength,1> mA;
    Eigen::Matrix<double,FilterLength,1> mB;
    
    //! Currently filtered sample
    double mCurrentSample;
    
    //! Rate of change of filtered samples
    double mCurrentSampleRate;
    
    //! Current sample average
    double mFilterAverage;
    
    //! Average dt of input accelerations
    double mdt;
};

/**************************************************************************//**
* Constructor.  Sets default parameters
* \arg \c params.  Parameter object.
******************************************************************************/
template<>
inline cStepDetector<11>::cStepDetector(boost::property_tree::ptree config)
{
    mMessageCount = 0;
    mStepCount = 0;
    mCurrentSample = 0.0;
    mCurrentSampleRate = 0.0;
    mFilterAverage = 0.0;
    mdt = 1.0 / config.get<unsigned int>("parameters.hardware.imus.xsensmti.rate");
    
    // Butterworth filter coefficients for low-pass filter with 1Hz cutoff and 100Hz sample rate
    // Matlab code version: [mB,mA]=butter(10,1/(100/2),'low');z=filter(b,a,acceleration);
    mA << 1.000000000000000e+00,
         -9.598354771449335e+00,
          4.146557927564451e+01,
         -1.061733549136486e+02,
          1.784400555584700e+02,
         -2.056795482768198e+02,
          1.646664856685542e+02,
         -9.041478757937934e+01,
          3.258510336315131e+01,
         -6.960335495590178e+00,
          6.691571710680245e-01;
       
    mB << 7.100440027607569e-16,
          7.100440027607569e-15,
          3.195198012423406e-14,
          8.520528033129082e-14,
          1.491092405797589e-13,
          1.789310886957107e-13,
          1.491092405797589e-13,
          8.520528033129082e-14,
          3.195198012423406e-14,
          7.100440027607569e-15,
          7.100440027607569e-16;
}

/**************************************************************************//**
* Assesses the internal message queue and emits and signal if step is detected.
* If less messages are available than the low pass window length, step is
* assumed false.
* \arg \c void
* \return \c void
******************************************************************************/
template<unsigned int FilterLength>
bool cStepDetector<FilterLength>::detect(double acceleration)
{
	bool stepdetected = false;
	mMessageCount++;
		
    mAccelerationBuffer.push_front(acceleration);
		
	if (mMessageCount >= mFilterLength)
	{
	    mCurrentSample = 0.0;
	  
	    // Apply "b" coefficients to all samples
        for (unsigned int k = 0; k < FilterLength; k++)
        {
            mCurrentSample += mB(k) * mAccelerationBuffer[k];
        }
        
        // Apply "a" coefficients to all but current sample
        for (unsigned int j = 0; j < FilterLength-1; j++)
        {
            mCurrentSample -= mA(j+1)*mFilterBuffer[j];
        }
        
        // Calculate rate of change of the low pass filtered acceleration
        mCurrentSampleRate = (mCurrentSample - mFilterBuffer.front()) / mdt;
         
        // If this is a zero (cumulative mean) crossing going downwards, a step has occurred
        if (mCurrentSample < mFilterAverage && mFilterBuffer.front() > mFilterAverage && mCurrentSampleRate < 0.0 && std::abs(mCurrentSampleRate) > 2.0)
        {
            stepdetected = true;
        }
        
        // Push on new filtered data
        mFilterBuffer.push_front(mCurrentSample);
        
        // Pop-off oldest data
	    mAccelerationBuffer.pop_back();
	    mFilterBuffer.pop_back();
	}
	else
	{
	    // Pad filtered output with zeros until we have sufficient inputs.
	    mFilterBuffer.push_front(0.0);
	}
	
	mFilterAverage += (mFilterBuffer.front() - mFilterAverage)/mMessageCount;
	
	return stepdetected;
}
#endif // CSTEPDETECTOR_H
