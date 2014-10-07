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
 * @file   cpdr.cpp
 * @brief  A pedestrian dead-reckoning system.  Integrates gyros and step counting
 * @author M. George
 */

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "cpdr.h"

using namespace BASIL;

/**************************************************************************//**
* cPDR Constructor.  Constant vectors and matrices (gravity,
* earth rate) are formed here.  PDR state is set to ALIGN initially.  PDR
* cannot be used until an alignment method is called.
* \arg \c params.  Boost::property_tree object and id of attached IMU.
******************************************************************************/
cPDR::cPDR(boost::property_tree::ptree config, unsigned int imuid) : mStepDetector(config)
{
    mImuId = imuid;
    
    mMessageCounter = 0;
    
    // Clear all the member vectors and matrices.
    mEarthRate           = CartesianVector::Zero();
    mGravity             = CartesianVector::Zero();
    mAlpha               = CartesianVector::Zero();
    mDeltaP              = CartesianVector::Zero();
    mDeltaV              = CartesianVector::Zero();
    mGravityReaction     = CartesianVector::Zero();

    mIdentity = CartesianMatrix::Identity();
    mA        = CartesianMatrix::Zero();
    
    mAlphaSkew     = SkewMatrix::Zero(); 
    mAlphaMagnitude = 0.0;

    mf1 = mf2 = mf3 = mf4 = 0.0;

    mCbn = RotationMatrix::Identity();

    mdt = 0;

    // INS initially stopped, time = 0, no messages and ready to calibrate.
    mTime = 0.0;
    mPreviousTime = 0.0;
    mPreviousEarthRateTime = 0.0;
    mAlignmentTime = config.get<double>("parameters.ins.alignmentperiod");
    mIMUCount = 0;
    mIMURate = config.get<unsigned int>("parameters.hardware.imus.xsensmti.rate");
    mEarthUpdateRate = config.get<unsigned int>("parameters.ins.earthupdaterate");
    
    if (mEarthUpdateRate < 0)
    {
        std::cerr << "Earth update rate must be >= 0" << std::endl;
        mApplyEarthRateCorrections = false;
        mEarthSubsample = 0;
    }
    else if (mEarthUpdateRate == 0)
    {
        mApplyEarthRateCorrections = false;
        mEarthSubsample = 0;
    }
    else
    {
        mApplyEarthRateCorrections = true;
        mEarthSubsample = mIMURate / mEarthUpdateRate;
    }
    
    // Get initial conditions
    mInitialHeading = config.get<double>("parameters.ins.initialheading")*BASIL::CONSTANTS::DEG_TO_RAD;
    mInitialLatitude = config.get<double>("parameters.ins.approximatelatitude")*BASIL::CONSTANTS::DEG_TO_RAD;
    
    // Step data
    mStepLength = config.get<double>("parameters.pdr.steplength");
    mStepIncrement = CartesianVector::Zero();
    
    // Clear initial INS data structure
    mInsData.Velocity    = CartesianVector::Zero();
    mInsData.Position    = Array::Zero();
    mInsData.Orientation = AngleArray::Zero();
    mInsData.InsMdl = BASIL::INS::GENERIC_INS;
    mInsData.Id = imuid;

    // Clear IMU calibration data structure
    mImuCalibrationData.AccelerationBias = CartesianVector::Zero();
    mImuCalibrationData.AccelerationScaleFactor = CartesianVector::Zero();
    mImuCalibrationData.AngularRateBias << 0.00414044772524217,0.00189662656009109,0.00726575764268722;
    mImuCalibrationData.AngularRateGBias = CartesianVector::Zero();
    mImuCalibrationData.AngularRateScaleFactor = CartesianVector::Zero();
    mImuCalibrationData.ImuMdl = BASIL::IMUS::GENERIC_IMU;
    mImuCalibrationData.Id = imuid;
    
    mCurrentState = ALIGNING;
}

/**************************************************************************//**
* cPDR Destructor
******************************************************************************/
cPDR::~cPDR()
{
}

/**************************************************************************//**
* Integrates the attitude forward using Savage type exact 
* integration. Ref: Savage, P.  Strapdown Analytics, 2nd Ed.  Chapters 7, 19 
* Steps are detected and position is dead-reckoned using heading data and
* average step length.
* \arg \c omega.  Vector of x,y,z gyro rate readings in rad/s
* \arg \c accel.  Vector of x,y,z accelerometer readings in m/s/s
* \arg \c time.   Time stamp of IMU readings
* \return \c void
******************************************************************************/
void cPDR::integratePVA(const cImuData& imudata)
{
    if (mCurrentState < cPDR::RUNNING)
    {
        std::cerr << "Error (cPDR): INS must be aligned before integrating." << std::endl;
	    return;
    }    

    // For the first message we receive predict using nominal IMU rate
    if (mMessageCounter == 0)
    {
        mdt = 1.0/double(mIMURate);
    }
    else
    {
        mdt = imudata.Time.getSeconds() - mPreviousTime;
    }
    
    // Catch an out of order packet here
    if (mdt < 0)
    {
        //Some kind of standard error here
        std::cerr << "Error (cPDR): dt is negative in integratePVA()" << std::endl;
        return;
    }

    if (mdt > 2.0/double(mIMURate))
    {
        // Some kind of standard error here
        std::cerr << "Warning (cPDR): dt (" << mdt << "s) is large in integratePVA()" << std::endl;
        // Reset the counters so this doesn't persist.
        mInsData.Time = imudata.Time;
        mPreviousTime = imudata.Time.getSeconds();
    	mMessageCounter++;
        //  Returning here is equivalent to assuming no motion during period of missing data.
        return;
    }

    // Get rotation vector
    mAlpha = imudata.AngularRate*mdt;
    mAlphaMagnitude = mAlpha.norm();
    
    // Catch zero rotation vector magnitudes which happen with stationary sensors and quantization
    if (mAlphaMagnitude > 0.0)
    {   
        // Skew symmetric matrix from rotation vector
        BASIL::MATH::skew(mAlpha, mAlphaSkew);
        
        // Integration constants
        mf1 = std::sin(mAlphaMagnitude)/mAlphaMagnitude;
        mf2 = (1.0 - std::cos(mAlphaMagnitude))/(mAlphaMagnitude*mAlphaMagnitude);
        mf3 = (1.0/(mAlphaMagnitude*mAlphaMagnitude))*(1.0-mf1);
        mf4 = (1.0/(mAlphaMagnitude*mAlphaMagnitude))*(0.5-mf2);
	
	    // Position update
	    if (mStepDetector.detect(imudata.Acceleration(BASIL::IMUZ)))
	    {
	        mStepIncrement(BASIL::NORTH) = mStepLength * std::cos(mInsData.Orientation(BASIL::YAW));
	        mStepIncrement(BASIL::EAST) = mStepLength * std::sin(mInsData.Orientation(BASIL::YAW));
	        mStepIncrement(BASIL::DOWN) = 0.0;
            mInsData.Position.noalias() += mStepIncrement;
            //std::cout << mMessageCounter << std::endl;
        }
        
        // Orientation update
        mA.noalias() = mIdentity + mf1*mAlphaSkew + mf2*mAlphaSkew*mAlphaSkew;
        mCbn *= mA;
        cbn2euler(mCbn,mInsData.Orientation);
    }
    else
    {
        // Position update
	    if (mStepDetector.detect(imudata.Acceleration(BASIL::IMUZ)))
	    {
	        mStepIncrement(BASIL::NORTH) = mStepLength * std::cos(mInsData.Orientation(BASIL::YAW));
	        mStepIncrement(BASIL::EAST) = mStepLength * std::sin(mInsData.Orientation(BASIL::YAW));
	        mStepIncrement(BASIL::DOWN) = 0.0;
            mInsData.Position.noalias() += mStepIncrement;
            //std::cout << mMessageCounter << std::endl;
        }
    }

    // Apply an earth rate correction at a lower frequency
    if ( mApplyEarthRateCorrections && (mMessageCounter % mEarthSubsample == 0) )
    {
        cPDR::earthRateCorrection(imudata.Time.getSeconds());
    }

    // Times are set here so that a user can bypass position and velocity integration
    // if desired, effectively using this class as a AHRS system with gyros only.
    mInsData.Time = imudata.Time;
    mPreviousTime = imudata.Time.getSeconds();

    mMessageCounter++;
}

/**************************************************************************//**
* Called at sub-intervals of the integreatePVA() function.  Adds correction 
* to current attitude to account for Earth rotation effects.  Position and velocity
* corrections are taken into account in integratePVA() and as a natural consequence
* of this update.  Not used if no latitude input is used in alignment call.
* \arg \c void
* \return \c void
******************************************************************************/
void cPDR::earthRateCorrection(const double time)
{   
    // First time in we use nominal earth correction rate for the dt.
    if (mPreviousEarthRateTime == 0.0)
    {
        mdt = 1.0 / (double)mEarthSubsample;
    }
    else
    {
        mdt = time - mPreviousEarthRateTime;
    }

    // Catch negative time increments
    if (mdt < 0)
    {
        //Some kind of standard error here
        std::cerr << "Error (cPDR): dt is negative in earthRateCorrection()" << std::endl;
        return;
    }

    mAlpha = -1.0*mdt*mEarthRate;
    mAlphaMagnitude = mAlpha.norm();
    mf1 = std::sin(mAlphaMagnitude)/mAlphaMagnitude;
    mf2 = (1.0 - std::cos(mAlphaMagnitude))/(mAlphaMagnitude*mAlphaMagnitude);
    BASIL::MATH::skew(mAlpha, mAlphaSkew);
    
    mA.noalias() = mIdentity + mf1*mAlphaSkew + mf2*mAlphaSkew*mAlphaSkew;
    mCbn = mA*mCbn;
    cbn2euler(mCbn,mInsData.Orientation);
    
    mPreviousEarthRateTime = time;
}

/**************************************************************************//**
* Local initial alignment with velocity and position in local navigation coordinates.  
* Roll and pitch angles are coarsely aligned from an input average acceleration vector 
* (2-3s recommended) with gravity.  Initial heading (North), velocity and position (Local) 
* are user inputs.  User must also input an approximate latitude.  The global heading and 
* approximate latitude allow reasonable gravity models and earth rate corrections to be fixed.
*\arg \c Pointer to 3x1 array of time averaged x,y,z accelerometer signals.  
* 		 Suggested averaging time 2-3s.  IMU must be stationary during this time!
* \return \c void
******************************************************************************/
void cPDR::localAlign(const CartesianVector& gravityReactionForce,  const double heading, const CartesianVector& initialUTMPosition, 
                      const CartesianVector& initialUTMVelocity, const double approximateLatitude)
{
    gravityAlignRollPitch(gravityReactionForce, mInsData.Orientation);
    mInsData.Orientation(BASIL::YAW) = heading;

    cPDR::euler2cbn(mInsData.Orientation,mCbn);

    mInsData.Velocity = initialUTMVelocity;
    mInsData.Position = initialUTMPosition;

    // Earth rate:
    getEarthRate(approximateLatitude, mEarthRate);
    
    // Gravity
    mGravity(BASIL::DOWN) = getGravityMagnitude(approximateLatitude);

    mMode = cPDR::LOCAL;
    // Earth rate corrections will be applied unless already turned off in constructor
    mApplyEarthRateCorrections = mApplyEarthRateCorrections && true;
    
    // Zero counters in case we need to re-align at some point
    mMessageCounter = 0;
    mdt = 0;
    mInsData.Time.setTime(TIME::FREE_RUNNING,0,0.0);
    mPreviousTime = 0.0;
    mPreviousEarthRateTime = 0.0;

    // We are now running and ready to call integrate*() functions
    mCurrentState = cPDR::RUNNING;
}

/**************************************************************************//**
* Local initial alignment with of roll and pitch angles only.  
* Roll and pitch angles are coarsely aligned from an input average acceleration vector 
* (2-3s recommended) with gravity.  Initial heading, velocity and position are all set to 0. 
* User must also input an approximate latitude to allow a coarse estimate of gravity.
*\arg \c Pointer to 3x1 array of time averaged x,y,z accelerometer signals.  
* 		 Suggested averaging time 2-3s.  IMU must be stationary during this time!
* \return \c void
******************************************************************************/
void cPDR::localAlign(const CartesianVector& gravityReactionForce, const double approximateLatitude)
{
    gravityAlignRollPitch(gravityReactionForce, mInsData.Orientation);
    
    euler2cbn(mInsData.Orientation,mCbn);
    
    // Earth rate:
    getEarthRate(approximateLatitude, mEarthRate);
    
    // Gravity
    mGravity(BASIL::DOWN) = getGravityMagnitude(approximateLatitude);
    
    mMode = cPDR::LOCAL;
    // Earth rate corrections will be applied unless already turned off in constructor
    mApplyEarthRateCorrections = mApplyEarthRateCorrections && true;
    
    // Zero counters in case we need to re-align at some point
    mMessageCounter = 0;
    mdt = 0;
    mInsData.Time.setTime(TIME::FREE_RUNNING,0,0.0);
    mPreviousTime = 0.0;
    mPreviousEarthRateTime = 0.0;
    
    // We are now running and ready to call integrate*() functions
    mCurrentState = cPDR::RUNNING;
}

/**************************************************************************//**
* Get the magnitude of gravity at our current latitude using a Defense Mapping 
* Agency model.  Reference:  Jay A. Farrell.  Aided Inertial Navigation, p.33.
* \arg \c latitude.  Reference to latitude data (radians)
* \arg \c gravity.  Reference to gravity output.
* \return \c void
******************************************************************************/
double cPDR::getGravityMagnitude(const double latitude)
{
    double gravity = BASIL::WGS84::EQUATORIAL_GRAVITY * 
                    ( (          1.0 + 0.0019318513530*std::sin(latitude)*std::sin(latitude)) / 
                      (std::sqrt(1.0 - 0.0066943800229*std::sin(latitude)*std::sin(latitude))) );
    return gravity;
}

/**************************************************************************//**
* Correct the current position, velocity and orientation with errors.
* \arg \c position.  Position.
* \arg \c velocity.  Velocity.
* \arg \c eulers.  Euler angles.
* \return \c void
******************************************************************************/
void cPDR::correctPVA(const CartesianVector& positionerror, 
                      const CartesianVector& velocityerror, 
                      const AngleArray& skewangles)
{
    mInsData.Velocity -= velocityerror;
    mInsData.Position -= positionerror;
    
    BASIL::MATH::skew(skewangles, mAlphaSkew); 
    mA = mIdentity - mAlphaSkew;
    mCbn = mA.inverse() * mCbn;
    
    cbn2euler(mCbn, mInsData.Orientation);
}

/**************************************************************************//**
* Get the components of earth rate at our current latitude.
* \arg \c latitude.  Reference to latitude data (radians)
* \arg \c gravity.  Pointer to earthRate vector for output.
* \return \c void
******************************************************************************/
void cPDR::getEarthRate(const double latitude, CartesianVector& earthRate)
{
    earthRate(NORTH) = BASIL::WGS84::EARTH_RATE * std::cos(latitude);
    earthRate(EAST) = 0;
    earthRate(DOWN) = -1.0 * BASIL::WGS84::EARTH_RATE*std::sin(latitude);
}

/**************************************************************************//**
* Get roll and pitch angles from a gravity vector.  Input vector should be averaged
* over a period of 2-3 seconds with a stationary IMU.  Aligns IMU x,y coordinates
* to Down gravity vector.  1 mg of accelerometer bias -> 1 mrad of roll, pitch error.
* Calibrate your accelerometer before averaging if possible.
* \arg \c latitude.  Reference to latitude data (radians)
* \arg \c gravity.  Pointer to earthRate vector for output.
* \return \c void
******************************************************************************/
void cPDR::gravityAlignRollPitch(const CartesianVector& gravityReactionForce, AngleArray& angles)
{    
    angles(ROLL) = std::atan2(-gravityReactionForce(BODYY),
                              -gravityReactionForce(BODYZ));
    angles(PITCH) = std::atan2(gravityReactionForce(BASIL::BODYX),
                                           std::sqrt((gravityReactionForce(BODYY) * gravityReactionForce(BODYY))
                                                    +(gravityReactionForce(BODYZ) * gravityReactionForce(BODYZ))));
}

/**************************************************************************//**
* Calculate a direction cosine matrix, Cbm, relating body frame to global North,
* East, Down frame from an Euler angle vector.  Cbm is propagated directly by
* integrate so this function is most useful at alignment time for initial Cbm.
* \arg \c Cbm.  Empty Cbm matrix to be filled.
* \arg \c type.  Euler angle vector.
* \return \c void
******************************************************************************/
void cPDR::euler2cbn(const AngleArray& eulers, CartesianMatrix& Cbn)
{
    double roll = eulers(0);
    double pitch = eulers(1);
    double yaw = eulers(2);
    
    Cbn << std::cos(yaw)*std::cos(pitch),
         std::cos(yaw)*std::sin(pitch)*std::sin(roll) - std::sin(yaw)*std::cos(roll),
         std::cos(yaw)*std::sin(pitch)*std::cos(roll) + std::sin(roll)*std::sin(yaw),
         std::cos(pitch)*std::sin(yaw),
         std::cos(roll)*std::cos(yaw) + std::sin(roll)*std::sin(pitch)*std::sin(yaw),
         std::cos(roll)*std::sin(pitch)*std::sin(yaw) - std::sin(roll)*std::cos(yaw),
         -std::sin(pitch),
         std::sin(roll)*std::cos(pitch),
         std::cos(roll)*std::cos(pitch);
}

/**************************************************************************//**
* Set the Euler angles from the current Cbm value
* \arg \c Cbm.  Current Cbm matrix.
* \return \c void
******************************************************************************/
void cPDR::cbn2euler(const RotationMatrix& Cbn, AngleArray& eulers)
{
    eulers(0) = std::atan2(Cbn(2,1),Cbn(2,2));
    eulers(1) = std::asin(-Cbn(2,0));
    eulers(2) = std::atan2(Cbn(1,0),Cbn(0,0));
}

/**************************************************************************//**
* Copy the current Cbm into the input reference.
* \arg \c out.  Empty Cbm matrix to be filled.
* \return \c void
******************************************************************************/
void cPDR::getCbn(RotationMatrix& out)
{
    out = mCbn;
}

/**************************************************************************//**
* Copy the current euler angles into the input reference.
* \arg \c out.  Empty position vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getEulers(AngleArray& out)
{
    cbn2euler(mCbn,out);
}

/**************************************************************************//**
* Copy the current velocity into the input reference.
* \arg \c out.  Empty velocity vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getVelocity(CartesianVector& out)
{
    out = mInsData.Velocity;
}

/**************************************************************************//**
* Copy the current position into the input reference.
* \arg \c out.  Empty position vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getPosition(CartesianVector& out)
{
    out = mInsData.Position;
}

/**************************************************************************//**
* Copy the current gravity vector into the input reference.
* \arg \c out.  Empty gravity vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getGravity(CartesianVector& out)
{
    out = mGravity;
}

/**************************************************************************//**
* Copy the current earth rate vector into the input reference.
* \arg \c out.  Empty earth rate vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getEarthRate(CartesianVector& out)
{
    out = mEarthRate;
}

/**************************************************************************//**
* Copy the current position, velocity and euler angles out.
* \arg \c position.  Empty position vector to be filled.
* \arg \c velocity.  Empty velocity vector to be filled.
* \arg \c eulers.  Empty euler angles vector to be filled.
* \return \c void
******************************************************************************/
void cPDR::getPVA(CartesianVector& position, CartesianVector& velocity, CartesianVector& eulers)
{
    position = mInsData.Position;
    velocity = mInsData.Velocity;
    cbn2euler(mCbn,eulers);
}

/**************************************************************************//**
* Set the current Cbm from the input field.
* \arg \c out.  Cbm matrix input.
* \return \c void
******************************************************************************/
void cPDR::setCbn(const RotationMatrix& in)
{
    mCbn = in;
    cbn2euler(mCbn, mInsData.Orientation);
}

/**************************************************************************//**
* Set the current euler angles from the input field.
* \arg \c out.  Euler angles (r,p,y) input.
* \return \c void
******************************************************************************/
void cPDR::setEulers(const AngleArray& in)
{
    mInsData.Orientation = in;
    euler2cbn(mInsData.Orientation, mCbn);
}

/**************************************************************************//**
* Set the current velocity from the input field.
* \arg \c out.  Velocity vector input.
* \return \c void
******************************************************************************/
void cPDR::setVelocity(const CartesianVector& in)
{
    mInsData.Velocity = in;
}

/**************************************************************************//**
* Set the current position from the input field.
* \arg \c out.  Position vector input.
* \return \c void
******************************************************************************/
void cPDR::setPosition(const CartesianVector& in)
{
    mInsData.Position = in;
}

/**************************************************************************//**
* Set the current position, velocity and euler angles.
* \arg \c position.  Position.
* \arg \c velocity.  Velocity.
* \arg \c eulers.  Euler angles.
* \return \c void
******************************************************************************/
void cPDR::setPVA(const CartesianVector& position, 
                  const CartesianVector& velocity, 
                  const AngleArray& eulers)
{
    mInsData.Position = position;
    mInsData.Velocity = velocity;
    mInsData.Orientation = eulers;
    euler2cbn(mInsData.Orientation, mCbn);
}

/**************************************************************************//**
* Set the current gravity vector
* \arg \c in.  Gravity in NED.
* \return \c void
******************************************************************************/
void cPDR::setGravity(const CartesianVector& in)
{
    mGravity = in;
}

/**************************************************************************//**
* Reset INS Counters so that a break in data does not cause a spike in the solution
* \arg \c void
* \return \c void
******************************************************************************/
void cPDR::resetCounters()
{
    // Zero counters in case we need to re-align at some point
    mMessageCounter = 0;
    mdt = 0;
    mTime = 0.0;
    mPreviousTime = 0.0;
    mPreviousEarthRateTime = 0.0;
}
