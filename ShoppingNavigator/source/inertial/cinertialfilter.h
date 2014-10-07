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
 * @file   cinertialfilter.h
 * @brief  A BASIL filter for blending IMU and place recognition inputs
 * @author M. George
 */

#ifndef CINERTIALFILTER_H
#define CINERTIALFILTER_H

#include <boost/property_tree/ptree.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Core>

#include "basil/basil.h"
#include "basil/types.h"
#include "basil/constants.h"

#include "basil/messages/sensors/cimumsg.h"
#include "basil/messages/sensors/cinsmsg.h"
#include "basil/messages/sensors/cplacerecognitionmsg.h"
#include "basil/messages/internal/cinserrormsg.h"
#include "basil/messages/internal/cfiltermsg.h"

#include "basil/filters/measurements/czuptdetector.h"
#include "basil/filters/measurements/cnedframevelocityerrormodel.h"
#include "basil/filters/measurements/cnedframepositionerrormodel.h"

#include "cpdrerrormodel.h"

using namespace BASIL;

template<template<typename> class FilterType, typename StateModel>
class cInertialFilter
{
public:
    cInertialFilter(StateModel& state, boost::property_tree::ptree config, std::string filtername);
    FilterType<StateModel> filter;
    void initialize(void);
    
    void operator()(cMessage<cImuMsg>& data);
    void operator()(cMessage<cInsMsg>& data);
    void operator()(cMessage<cPlaceRecognitionMsg>& data);
    
    void feedbackInsErrors(void);
    
    //! Zupt requirements
    cZuptDetector mZuptDetector;  
    cNEDFrameVelocityErrorModel<StateModel> mZuptModel;
    cNEDFramePositionErrorModel<StateModel> mPositionModel;
    
    typename StateModel::State mInitialStateMean;
    typename FilterType<StateModel>::Covariance mInitialCovariance;

    // Filter data save 
    cFilterMsg mFilterData;
    
    bool mInsInitialized;
    bool mPredicted;
    unsigned long mImuCount;
    unsigned int mPredictStride;
    unsigned int mImuId;
    
    //! INS message to carry error feedback
    cInsErrorMsg mInsErrors;
    
    //! Mutex to lock-out INS and Place recognition thread clashes
    boost::mutex mMutex;
};

/**************************************************************************
*
* FULL INERTIAL NAVIGATION FILTER GENERIC IMPLEMENTATIONS
*
**************************************************************************/

// This is well ugly, but here is the constructor for a 9 State Kalman Filter
template<>
cInertialFilter< BASIL::cKalmanFilter, BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> >::
cInertialFilter(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>& state, boost::property_tree::ptree config, std::string filtername):
mZuptDetector(config,filtername), mZuptModel(config,filtername,"zupts",6), mPositionModel(config,filtername,"place-recognition",3), filter(state,config,filtername)
{
    // Get initial parameters variances from config file by looking up state names in state model.
    mInitialCovariance.setZero();
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::ROLL,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::ROLL) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.roll"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PITCH,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PITCH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.pitch"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::YAW,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::YAW) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.heading"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PNORTH,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PNORTH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.north"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PEAST,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.east"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PDOWN,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PDOWN) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.down"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VNORTH,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VNORTH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.northvel"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VEAST,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.eastvel"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VDOWN,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VDOWN) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.downvel"),2);

    // Initial mean
    mInitialStateMean = BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::State::Zero();
    
    // Set the constant components of the INS error feedback
    mInsErrors.HardwareId = 0;
    mInsErrors.DeviceId = 0;
    mInsErrors.ImuMdl = BASIL::IMU::XSENS_MTI;
    
    // Initial conditions
    mInsInitialized = false;
    mPredicted = false;
    mImuCount = 0;
    mPredictStride = config.get<unsigned int>("parameters.filter." + filtername + ".predictstride");
    mImuId = 0;
    
    // Set the filter running
    filter.mFilterState == cKalmanFilter< BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> >::CONSTRUCTED;

    initialize();
}

// This is well ugly, but here is the constructor for a 15 State Kalman Filter
template<>
cInertialFilter< BASIL::cKalmanFilter, BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE> >::
cInertialFilter(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>& state, boost::property_tree::ptree config, std::string filtername):
mZuptDetector(config,filtername), mZuptModel(config,filtername,"zupts",6), mPositionModel(config,filtername,"place-recognition",3), filter(state,config,filtername)
{
    // Get initial parameters variances from config file by looking up state names in state model.
    mInitialCovariance.setZero();
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ROLL,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ROLL) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.roll"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PITCH,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PITCH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.pitch"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::YAW,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::YAW) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.heading"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PNORTH,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PNORTH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.north"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PEAST,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.east"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PDOWN,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PDOWN) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.down"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VNORTH,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VNORTH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.northvel"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VEAST,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.eastvel"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VDOWN,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VDOWN) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.downvel"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROXBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROXBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.gyroxbias"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROYBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROYBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.gyroybias"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROZBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROZBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.gyrozbias"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELXBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELXBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.accelxbias"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELYBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELYBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.accelybias"),2);
    mInitialCovariance(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELZBIAS,BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELZBIAS) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.accelzbias"),2);

    // Initial mean
    mInitialStateMean = BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::State::Zero();

    // Set the constant components of the INS error feedback
    mInsErrors.HardwareId = 0;
    mInsErrors.DeviceId = 0;
    mInsErrors.ImuMdl = BASIL::IMU::XSENS_MTI;

    // Initial conditions
    mInsInitialized = false;
    mPredicted = false;
    mImuCount = 0;
    mPredictStride = config.get<unsigned int>("parameters.filter." + filtername + ".predictstride");
    mImuId = 0;

    // Set the filter running
    filter.mFilterState == cKalmanFilter< BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE> >::CONSTRUCTED;

    initialize();
}

template<template<typename> class FilterType, typename StateModel>
inline void cInertialFilter<FilterType,StateModel>::initialize(void)
{
    filter.initialize(mInitialStateMean,mInitialCovariance);

    mInsInitialized = false;
    mPredicted = false;
    mImuCount = 0;
}

/********************************************************************************
*  IMU SECTION:
*  Incoming IMU data triggers filter predictions and possibly zupt updates
*********************************************************************************/
template<template<typename> class FilterType, typename StateModel>
inline void cInertialFilter<FilterType,StateModel>::operator()(cMessage<cImuMsg>& data)
{
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    if (filter.mFilterState != cKalmanFilter<StateModel>::RUNNING)
    {
        return;
    }
    
    mImuCount++;
    
    // Predict with IMU data
    if (mImuCount % mPredictStride == 0)
    {
        filter.predict(data.Time);
        mPredicted = true;
    }
    
    // Check for zupts and update if found
    if (data.derived().DeviceId == mImuId)
    {
        if (mZuptDetector.detect(data.derived().Acceleration) && mPredicted)
        {
            filter.update(mZuptModel);
            feedbackInsErrors();
            mPredicted = false;
        }
    }
}

/********************************************************************************
 *  INS SECTION:
 *  Incoming INS data simply tells the filter that it can start since the INS
 *  must now be aligned and operating.
 *********************************************************************************/
template<template<typename> class FilterType, typename StateModel>
inline void cInertialFilter<FilterType,StateModel>::operator()(cMessage<cInsMsg>& data)
{
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    // Can the filter start?  Only when the INS is reporting  
    if (filter.mFilterState != cKalmanFilter<StateModel>::RUNNING)
    {
        if (data.derived().DeviceId == mImuId)
        {
            mInsInitialized = true;
            filter.mFilterState = cKalmanFilter<StateModel>::RUNNING;
        }
    }
    
    // We need to save INS velocity for the zupt measurement residuals
    if (data.derived().DeviceId == mImuId)
    {
        mZuptModel.mMeasurement = data.derived().Velocity;
        mPositionModel.mMeasurement = data.derived().Position;
    }
}

/********************************************************************************
 *  VISUAL DETECTION SECTION:
 *  Incoming Landmark data tells the filter that it is currently at a certain
 *  recognizable location.
 *********************************************************************************/
template<template<typename> class FilterType, typename StateModel>
inline void cInertialFilter<FilterType,StateModel>::operator()(cMessage<cPlaceRecognitionMsg>& data)
{   
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    // Do nothing if we're not yet running
    if (filter.mFilterState != cKalmanFilter<StateModel>::RUNNING)
    {
        return;
    }

    if (data.derived().Probability < 0.995)
    {
        return;
    }

    std::cout << "!!!!!!!!!!**********CORRECTING WITH MAP MATCH*****!!!!!!!!" << std::endl;
    // Perform position correction
    mPositionModel.mMeasurement -= data.derived().Location;
    filter.update(mPositionModel);
    feedbackInsErrors();
    mPredicted = false;
}

/********************************************************************************
 *  FEEDBACK SECTION:
 *  An InsErrors object is created and populated and then it's subscribers are called
 *********************************************************************************/
template<template<typename> class FilterType, typename StateModel>
inline void cInertialFilter<FilterType,StateModel>::feedbackInsErrors(void)
{
    // Left feedback
    mInsErrors.DeviceId = mImuId;
    mInsErrors.PositionError = filter.mStateMean.template segment<3>(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::PNORTH);
    mInsErrors.VelocityError = filter.mStateMean.template segment<3>(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::VNORTH);
    mInsErrors.OrientationError = filter.mStateMean.template segment<3>(BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>::ROLL);
    cInsErrorMsg::notify(mInsErrors);

    // Reset errors
    filter.mStateMean = StateModel::State::Zero();
}

/********************************************************************************
 *  FEEDBACK SECTION:
 *  An InsErrors object is created and populated and then it's subscribers are called
 *********************************************************************************/
template<>
inline void cInertialFilter< BASIL::cKalmanFilter, BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE> >::feedbackInsErrors(void)
{
    // Left feedback
    mInsErrors.DeviceId = mImuId;
    mInsErrors.PositionError = filter.mStateMean.segment<3>(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::PNORTH);
    mInsErrors.VelocityError = filter.mStateMean.segment<3>(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::VNORTH);
    mInsErrors.OrientationError = filter.mStateMean.segment<3>(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ROLL);
    mInsErrors.GyroBiasError =  filter.mStateMean.segment<3>(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::GYROBIAS);
    mInsErrors.AccelerometerBiasError =  filter.mStateMean.segment<3>(BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::ACCELBIAS);
    cInsErrorMsg::notify(mInsErrors);

    // Reset errors
    filter.mStateMean = BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE>::State::Zero();
}

/**************************************************************************
*
* PDR NAVIGATION FILTER SPECIALIZATIONS
*
**************************************************************************/

// This is well ugly, but here is the constructor for a PDR Kalman Filter.  DO NOT use the zupt model here as the state has no velocity components.
template<>
cInertialFilter<BASIL::cKalmanFilter, BASIL::cPdrErrorModel>::
cInertialFilter(BASIL::cPdrErrorModel& state, boost::property_tree::ptree config, std::string filtername):
mZuptDetector(config, filtername), mZuptModel(config,filtername,"zupts",0), mPositionModel(config,filtername,"place-recognition",3), filter(state,config,filtername)
{
    // Get initial parameters variances from config file by looking up state names in state model.
    filter.mStateCovariance.setZero();
    filter.mStateCovariance(BASIL::cPdrErrorModel::ROLL,BASIL::cPdrErrorModel::ROLL) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.roll"),2);
    filter.mStateCovariance(BASIL::cPdrErrorModel::PITCH,BASIL::cPdrErrorModel::PITCH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.pitch"),2);
    filter.mStateCovariance(BASIL::cPdrErrorModel::YAW,BASIL::cPdrErrorModel::YAW) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.heading"),2);
    filter.mStateCovariance(BASIL::cPdrErrorModel::PNORTH,BASIL::cPdrErrorModel::PNORTH) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.north"),2);
    filter.mStateCovariance(BASIL::cPdrErrorModel::PEAST,BASIL::cPdrErrorModel::PEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.east"),2);
    filter.mStateCovariance(BASIL::cPdrErrorModel::PEAST,BASIL::cPdrErrorModel::PEAST) = std::pow(config.get<double>("parameters.filter." + filtername + ".initialuncertainty.down"),2);

    // Set the constant components of the INS error feedback
    mInsErrors.ImuMdl = BASIL::IMU::XSENS_MTI;
    
    // Initial conditions
    mInsInitialized = false;
    mPredicted = false;
    mImuCount = 0;
    mPredictStride = config.get<unsigned int>("parameters.filter." + filtername + ".predictstride");
    mImuId = 0;
    
    // Set the filter running
    filter.mFilterState == cKalmanFilter<BASIL::cPdrErrorModel>::CONSTRUCTED;
}

/********************************************************************************
 *  FEEDBACK SECTION:
 *  An InsErrors object is created and populated and then it's subscribers are called
 *********************************************************************************/
template<>
inline void cInertialFilter<BASIL::cKalmanFilter,BASIL::cPdrErrorModel>::feedbackInsErrors(void)
{
    // Set feedback message fields
    mInsErrors.DeviceId = mImuId;
    mInsErrors.PositionError.segment<2>(0) = filter.mStateMean.segment<2>(BASIL::cPdrErrorModel::PNORTH);
    //mInsErrors.OrientationError = filter.mStateMean.segment<3>(BASIL::cPdrErrorModel::ROLL);
    //mInsErrors.OrientationError(2) *= -1.0;
    cInsErrorMsg::notify(mInsErrors);
    
    // Reset errors
    filter.mStateMean = BASIL::cPdrErrorModel::State::Zero();
}

/********************************************************************************
*  IMU SECTION:
*  Incoming IMU data triggers filter predictions and possibly zupt updates
*********************************************************************************/
template<>
inline void cInertialFilter<BASIL::cKalmanFilter,BASIL::cPdrErrorModel>::operator()(cMessage<cImuMsg>& data)
{
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    if (filter.mFilterState != cKalmanFilter<BASIL::cPdrErrorModel>::RUNNING)
    {
        return;
    }
    
    mImuCount++;
    
    // Predict with IMU data
    if (mImuCount % mPredictStride == 0)
    {
        // Predict in the filter
        filter.predict(data.Time);
        
        // Save filter data
        mFilterData.DeviceId = 0;
        mFilterData.Time = data.Time;
        mFilterData.Mean = filter.mStateMean;
        mFilterData.Covariance = filter.mStateCovariance.diagonal();
        mFilterData.Residual = mPositionModel.mMeasurement - mPositionModel.mPrediction;
        mFilterData.ResidualCovariance = filter.mS.diagonal();
        cFilterMsg::notify(mFilterData);
            
        mPredicted = true;
    }
}

/********************************************************************************
 *  INS SECTION:
 *  Incoming INS data simply tells the filter that it can start since the INS
 *  must now be aligned and operating.
 *********************************************************************************/
template<>
inline void cInertialFilter<BASIL::cKalmanFilter,BASIL::cPdrErrorModel>::operator()(cMessage<cInsMsg>& data)
{
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    // Can the filter start?  Only when the INS is reporting  
    if (filter.mFilterState != cKalmanFilter<BASIL::cPdrErrorModel>::RUNNING)
    {
        if (data.derived().DeviceId == mImuId)
        {
            mInsInitialized = true;
            filter.mFilterState = cKalmanFilter<BASIL::cPdrErrorModel>::RUNNING;
        }
    }
    
    // We need to save INS position for the landmark measurement residuals
    if (data.derived().DeviceId == mImuId)
    {
        mPositionModel.mMeasurement = data.derived().Position;
    }
}

/********************************************************************************
 *  VISUAL DETECTION SECTION:
 *  Incoming Landmark data tells the filter that it is currently at a certain
 *  recognizable location.
 *********************************************************************************/
template<>
inline void cInertialFilter<BASIL::cKalmanFilter,BASIL::cPdrErrorModel>::operator()(cMessage<cPlaceRecognitionMsg>& data)
{   
    boost::mutex::scoped_lock scoped_lock(mMutex);
    
    // Do nothing if we're not yet running
    if (filter.mFilterState != cKalmanFilter<BASIL::cPdrErrorModel>::RUNNING)
    {
        return;
    }
    // Perform position correction
    mPositionModel.mMeasurement -= data.derived().Location;
    filter.update(mPositionModel);
    feedbackInsErrors();
}

#endif // CINERTIALFILTER_H
