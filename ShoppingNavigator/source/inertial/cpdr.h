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
 * @file   cpdr.h
 * @brief  A pedestrian dead-reckoning system.  Integrates gyros and step counting
 * @author M. George
 */

#ifndef CPDR_H
#define CPDR_H

#include <boost/property_tree/ptree.hpp>

#include "basil/basil.h"
#include "basil/types.h"
#include "basil/constants.h"
#include "basil/messages/sensors/cimudata.h"
#include "basil/messages/sensors/cinsdata.h"
#include "basil/messages/sensors/cinserrordata.h"
#include "basil/messages/sensors/cimucalibrationdata.h"

#include "cstepdetector.h"

namespace BASIL
{
    /**************************************************************************//**
    * \class cPDR
    * PDR class.  Contains data structures and computations for a pedestrian dead 
    * reckoning system.  Orientation is tracked by 3-axis gyroscopes using full
    * AHRS implementation.  Steps are detected and counted and position dead reckoned
    * in heading direction.
    ******************************************************************************/
    class cPDR
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Constructor with nominal IMU data rate in Hz. and desired earth update rate in Hz.
        cPDR(boost::property_tree::ptree config, unsigned int imuid = 0);
        
        //! Destructor
        ~cPDR();
        
        //! Functor template for receiving BASIL callbacks
        template<typename T> void operator()(cData<T>& data);
        
        //! Two possible states for PDR system
        enum pdrState{ALIGNING,RUNNING};
        
        //! PDR mode is global or local.
        enum pdrMode{GLOBAL,LOCAL};
        
        //! Global alignment.  Use gravity to align roll and pitch angles, input initial heading, velocity and position (UTM)
        void globalAlign(const CartesianVector& gravityReactionForce,  const double heading, const CartesianVector& initialUTMPosition, 
                         const CartesianVector& initialUTMVelocity, const unsigned int UTMZone, const BASIL::Hemisphere hemisphere);
        
        //! Local alignment.  Use gravity to align roll and pitch angles, input initial heading (Global), velocity and position (Local)
        void localAlign(const CartesianVector& gravityReactionForce,  const double heading, const CartesianVector& initialUTMPosition, 
                        const CartesianVector& initialUTMVelocity, const double approximateLatitude);
        
        //! Local alignment.  Use gravity to align roll and pitch angles, zero heading, velocity, position assumed
        void localAlign(const CartesianVector& gravityReactionForce, const double approximateLatitude);
        
        //! Perform 6-dof dead reckoning, position and attitude (orientation).
        void integratePVA(const cImuData& imudata);
        
        //! Perform earth rate orientation correction update (advanced users)
        void earthRateCorrection(const double time);
        
        //! Cbn (C_body^navigation) getter function
        static void cbn2euler(const RotationMatrix& Cbn, AngleArray& eulers);
        
        //! Convert euler angles to Cbn matrix
        static void euler2cbn(const AngleArray& euler, RotationMatrix& Cbn);
        
        //! Convert rodrigues parameters to Cbn matrix
        static void rodrigues2cbn(const AngleArray& rodrigues, RotationMatrix& Cbn);
        
        //! Get Euler angle rate matrix
        static void getEulerRateMatrix(const AngleArray& euler, CartesianMatrix& E);
        
        //! Element-wise partial derivate of Cbn with respect to euler angles
        static void dCbndEuler(const AngleArray& euler, CartesianMatrix& dCbndRoll, CartesianMatrix& dCbndPitch, CartesianMatrix& dCbndYaw);
        
        //! Element-wise partial derivate of E (Euler rate matrix) with respect to euler angles
        static void dEdEuler(const AngleArray& euler, CartesianMatrix& dCbndRoll, CartesianMatrix& dCbndPitch, CartesianMatrix& dCbndYaw);
        
        //! Get jacobian of orientation alignment w.r.t. average accelerations
        void getAlignmentJacobian(const CartesianVector& gravityReactionForce, CartesianMatrix& jacobian);
        
        //! Get conversion from euler angle errors to Cbn skew error form
        void getEulerError2CbnError(double* out);
        
        //! Euler angle getter function
        void getEulers(AngleArray& out);
        
        //! Cbn getter function
        void getCbn(RotationMatrix& out);
        
        //! Velocity getter function.
        void getVelocity(CartesianVector& out);
        
        //! Position getter function
        void getPosition(CartesianVector& out);
        
        //! Gravity getter functions
        void getGravity(CartesianVector& out);
        
        //! Earth rate getter functions
        void getEarthRate(CartesianVector& out);
        
        //! Get position, velocity and attitude
        void getPVA(CartesianVector& position, CartesianVector& velocity, AngleArray& eulers);
        
        //! Get gravity magnitude at current latitude using DMA model
        double getGravityMagnitude(const double latitude);
        
        //! Get gravity magnitude at current latitude using simple trig. model
        void getEarthRate(const double latitude, CartesianVector& earthRate);
        
        //! Euler angle setter function
        void setEulers(const AngleArray& in);
        
        //! Cbn setter function
        void setCbn(const RotationMatrix& in);
        
        //! Velocity setter function.
        void setVelocity(const CartesianVector& in);
        
        //! Position setter functions
        void setPosition(const CartesianVector& in);
        
        //! Gravity setter functions
        void setGravity(const CartesianVector& in);
        
        //! Get position, velocity and attitude
        void setPVA(const CartesianVector& position, const CartesianVector& velocity, const AngleArray& eulers);
        
        //! Correct position, velocity and attitude
        void correctPVA(const CartesianVector& positionerror, const CartesianVector& velocityerror, const CartesianVector& skewangles);
        
        //! Get Time
        double getTime(){return mTime;}
        
        //! Reset counter so a break in data does no cause a large jump.
        void resetCounters();
        
        /********************************************************************************
         *   BASIC INS FUNCTIONS
         *   This section contains member functions for helpful operations that the user
         *   will have no need for
         *********************************************************************************/
        void gravityAlignRollPitch(const CartesianVector& gravityReactionForce, AngleArray& angles);
        
        /********************************************************************************
         *   BASIC INS MEMBERS
         *   This section contains member variables for timing, update counters etc.
         *********************************************************************************/
        //! Id number of associated IMU
        unsigned int mImuId;
        
        //! Time associated with current INS states
        double mTime;
        
        //! Previous time stamp so we can form dt.
        double mPreviousTime;
        
        //! Time difference
        double mdt;
        
        //! Previous earth rate correction time stamp so we can form dt.
        double mPreviousEarthRateTime;
        
        //! Count how many IMU messages received from hardware.
        unsigned long mMessageCounter;
        
        //! Nominal update rate for the IMU in use
        unsigned int mIMURate;
        
        //! Counter for number of incoming imu data packets
        unsigned long mIMUCount;
        
        //! How often to update orientation with earth rate (samples)
        unsigned int mEarthUpdateRate;
        
        //! How often to update orientation with earth rate (samples)
        unsigned int mEarthSubsample;
        
        //! How long to average during quasi-stationary alignment
        double mAlignmentTime;
        
        /********************************************************************************
         *   INS POSITION, VELOCITY, ORIENTATION
         *   This section contains member variables for position, velocity and orientation
         *   tracking.
         *********************************************************************************/
        //! BASIL format INS data, containing position, velocity and orientation data
        cInsData mInsData;

        //! BASIL format IMU calibration data
        cImuCalibrationData mImuCalibrationData;

        //! Coordinate transformation matrix from body to world coordinates
        RotationMatrix mCbn;

        //! Position increment for a single step
        CartesianVector mStepIncrement;
        double mStepLength;
        cStepDetector<11> mStepDetector;

        //! Earth Rate vector.  Constructed from cConstants
        CartesianVector mEarthRate;
        
        //! Gravity vector.
        CartesianVector mGravity;
        
        //! Current state of the PDR
        pdrState mCurrentState;
        
        //! Current mode of the PDR
        pdrMode mMode;
        
        //! UTM zone.  Current assumption that we do not traverse far enough to change zones
        unsigned int mUTMZone;
        
        //! Which hemisphere are we operating in.  Assumption is it does not change
        BASIL::Hemisphere mHemisphere;
        
        //! Should we do earth rate corrections
        bool mApplyEarthRateCorrections;
        
        //! Initial heading
        double mInitialHeading;
        
        //! Initial latitude (may be approx.)
        double mInitialLatitude;
        /********************************************************************************
         *   INS INTERMEDIATES AND HELPERS
         *   This section contains member variables for intermediate calculations and helpers
         *   BLAS functions cannot used variables with overlapping memory so intermediates 
         *   are necessary.
         *********************************************************************************/   
        //! Attitude update matrix.  Post-multiplies Cbn.
        CartesianMatrix mA;
        
        //! Rotation vector
        AngleArray mAlpha;
        double mAlphaMagnitude;

        //! Skew symmetric matrix of rotation vector
        SkewMatrix mAlphaSkew;
        
        //! Rotation vector helper functions
        double mf1;
        double mf2;
        double mf3;
        double mf4;

        //! Change in position increment
        CartesianVector mDeltaP;

        //! Change in velocity increment
        CartesianVector mDeltaV;
        
        //! Identity matrix
        CartesianMatrix mIdentity;
        
        //! Gravity reaction force average during stationary alignment
        CartesianVector mGravityReaction;
    };
    
    /********************************************************************************
     *   PDR SECTION - IMU DATA CALLBACK:
     *   Initial calibration, alignment and continuing dead reckoning is done here.
     *********************************************************************************/   
    template<> inline void cPDR::operator()(cData<cImuData>& data)
    {
        if (data.derived().Id == mImuId)
        {
            mIMUCount++;
        
            switch (mCurrentState)
            {
                case ALIGNING:
                {
                    // Compute incremental average.
                    mGravityReaction += (data.derived().Acceleration - mGravityReaction)/mIMUCount;
                    //mImuCalibrationData.AngularRateBias += (data.derived().AngularRate - mImuCalibrationData.AngularRateBias)/mIMUCount;
                    if ( mIMUCount == (mAlignmentTime * mIMURate) )
                    {
                        // Do local alignment, roll and pitch from average acceleration, initial heading, pos and velocity set manually.  
                        // Gravity and earth rate are set from latitude.
                        localAlign(mGravityReaction,mInitialHeading,CartesianVector::Zero(),CartesianVector::Zero(),mInitialLatitude);
                    }
                    break;
                }
                case RUNNING:
                {
                    // Calibrate IMU data with best estimate of errors
                    data.derived().Acceleration -= mImuCalibrationData.AccelerationBias;
                    data.derived().AngularRate -= mImuCalibrationData.AngularRateBias;

                    // Integrate IMU data in the INS.
                    integratePVA(data.derived());
                    
                    // Notify subscribers
                    cInsData::notify(mInsData);
                    break;
                }
            }
        }
    }
    
    /********************************************************************************
     *   PDR ERROR SECTION:
     *   Feedback from a filter with PDR errors
     *********************************************************************************/   
    template<> inline void cPDR::operator()(cData<cInsErrorData>& data)
    {
        if (data.derived().Id == mImuId)
        {
            // Some of the errors might not be estimated by a filter, in which case they will be 0.0.
            mImuCalibrationData.Time = mInsData.Time;
            mImuCalibrationData.AngularRateBias -= data.derived().GyroBiasError;
            mImuCalibrationData.AccelerationBias -= data.derived().AccelerometerBiasError;
            correctPVA(data.derived().PositionError,data.derived().VelocityError,data.derived().OrientationError);
            if (mIMUCount % 100 == 0)
            {
                cImuCalibrationData::notify(mImuCalibrationData);
            }
        }
    }

    /********************************************************************************
     *   IMU CALIBRATION SECTION:
     *   Feedback from an external calibration source.
     *********************************************************************************/
    template<> inline void cPDR::operator()(cData<cImuCalibrationData>& data)
    {
        if (data.derived().Id == mImuId)
        {
            mImuCalibrationData = data.derived();
        }
    }
}
#endif // CPDR_H
