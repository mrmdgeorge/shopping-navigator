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
 * @file   cpdrerrormodel.h
 * @brief  A pedestrian dead-reckoning error model for filtering
 * @author M. George
 */

#ifndef CPDRERRORMODEL_H
#define CPDRERRORMODEL_H

#include <cmath>

#include <boost/property_tree/ptree.hpp>

#include <Eigen/Core>

#include "basil/types.h"
#include "basil/messages/cmessage.h"
#include "basil/messages/sensors/cimumsg.h"
#include "basil/messages/sensors/cinsmsg.h"
#include "basil/inertial/cins.h"
#include "basil/filters/cstatemodel.h"

namespace BASIL
{
    /**************************************************************************//**
    * \class cPdrErrorModel
    * Extends cStateModel to make a pedestrian dead-reckoning (PDR) error model
    * that integrates gyros for orientation tracking and counts steps for position
    * tracking.
    ******************************************************************************/
    class cPdrErrorModel : public cStateModel<cPdrErrorModel,6,0,5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Some typedefs from the base class
        typedef typename cStateModel<cPdrErrorModel,6,0,5>::State State;
        typedef typename cStateModel<cPdrErrorModel,6,0,5>::Control Control;
        typedef typename cStateModel<cPdrErrorModel,6,0,5>::NoiseCovariance NoiseCovariance;
        
        //! Default constructor
        cPdrErrorModel(boost::property_tree::ptree config, std::string filtername);
        
        //! Constructor with state and noise input dimensions
        ~cPdrErrorModel(){}
        
        //! Functor template for receiving BASIL callbacks
        template<typename T>  void operator()(cMessage<T>& in);
        
        //! State enums
        enum{ROLL, PITCH, YAW,
             PNORTH, PEAST, PDOWN};
        
        enum{ORIENTATION = 0,
             POSITION = 3};
        
        //! Noise enums
        enum{GYROXNOISE, GYROYNOISE, GYROZNOISE,
             FORWARDNOISE,LATERALNOISE};
        
        enum{GYRONOISE = 0,
             TRANSLATIONNOISE = 3};
        
        //! Define a state transition function f(k) = f(x(k-1),u(k),w(k))
        void stateTransition(const State& oldstate, const Control& controls, State& newstate);
        
        //! Define a state time derivative function f_dot = f(x,u,w)
        void stateDerivative(const State& state, const Control& controls, State& derivative);
        
        //! Define a state jacobian function
        template <typename StateDerived, typename JacobianDerived>
        void stateJacobian(const Eigen::MatrixBase<StateDerived>& state, const Eigen::MatrixBase<JacobianDerived>& jacobian);
        
        //! Define a noise jacobian function
        template <typename StateDerived, typename JacobianDerived>
        void noiseJacobian(const Eigen::MatrixBase<StateDerived>& state, const Eigen::MatrixBase<JacobianDerived>& jacobian);        
        
        //! Noise covariance matrix, Q
        NoiseCovariance Q;
        
        //! Required data for generating state and noise jacobians
        RotationMatrix  mCbn;
        double mYaw;
        double mStepLength;
        
        double mNominalRate;
    };

    /*********************************************************************************//**
    * Default Constructor
    * \return \c void
    **********************************************************************************/
    cPdrErrorModel::cPdrErrorModel(boost::property_tree::ptree config, std::string filtername)
    {
        // Nominal prediction rate used to turn discrete step events into continuous differential equation
        mNominalRate = config.get<double>("parameters.filter." + filtername + ".rate");
         
        // Direction cosine matrix transforming b to n frame
        mCbn = RotationMatrix::Zero();
        mYaw = 0.0;
        mStepLength = config.get<double>("parameters.pdr.steplength");
        
        // Noise covariance matrix.  E[v*v'].  Initialized to diagonal, imu noise inputs are uncorrelated.
        Q = NoiseCovariance::Zero();
        Q(GYROXNOISE,GYROXNOISE) = std::pow(config.get<double>("parameters.hardware.imus.xsensmti.uncertainty.gyroxnoise"),2);
        Q(GYROYNOISE,GYROYNOISE) = std::pow(config.get<double>("parameters.hardware.imus.xsensmti.uncertainty.gyroynoise"),2);
        Q(GYROZNOISE,GYROZNOISE) = std::pow(config.get<double>("parameters.hardware.imus.xsensmti.uncertainty.gyroznoise"),2);
        Q(FORWARDNOISE,FORWARDNOISE) = std::pow(config.get<double>("parameters.hardware.imus.xsensmti.uncertainty.forwardnoise"),2);
        Q(LATERALNOISE,LATERALNOISE) = std::pow(config.get<double>("parameters.hardware.imus.xsensmti.uncertainty.lateralnoise"),2);
    }

    /*********************************************************************************//**
    * cPdrErrorModel::stateDerivative
    * Error states are constant (zero) during prediction cycle so no state transition is needed.
    * \return \c void
    **********************************************************************************/
    void cPdrErrorModel::stateTransition(const State& oldstate, const Control& controls, State& newstate)
    {
        //newstate = oldstate;
    }

    /*********************************************************************************//**
    * cPdrErrorModel::stateDerivative
    * Error states are zero during prediction cycle so no state prediction is needed.  
    * Hence state derivative is set to zero to save computation.
    * \return \c void
    **********************************************************************************/
    void cPdrErrorModel::stateDerivative(const State& state, const Control& controls, State& derivative)
    {
        //derivative = State::Zero();
    }

    /*********************************************************************************//**
    * cInsErrorModel::stateJacobian
    * \return \c void
    **********************************************************************************/
    template<typename StateDerived, typename JacobianDerived>
    void cPdrErrorModel::stateJacobian(const Eigen::MatrixBase<StateDerived>& state_, const Eigen::MatrixBase<JacobianDerived>& jacobian_)
    {
        // Get a non-const reference, hack for Eigen to allow matrix.block() inputs. See:
        // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
        // Eigen::MatrixBase<StateDerived>& state = const_cast< Eigen::MatrixBase<StateDerived>& >(state_);
        Eigen::MatrixBase<JacobianDerived>& jacobian = const_cast< Eigen::MatrixBase<JacobianDerived>& >(jacobian_);
        
        // POSITION ROW
        jacobian(PNORTH,YAW) = -mStepLength * std::sin(mYaw);
        jacobian(PEAST,YAW) = mStepLength * std::cos(mYaw);
    }
    
    /*********************************************************************************//**
    * c9StateInsErrorModel::noiseJacobian
    * \return \c void
    **********************************************************************************/
    template<typename StateDerived, typename JacobianDerived>
    void cPdrErrorModel::noiseJacobian(const Eigen::MatrixBase<StateDerived>& state_, const Eigen::MatrixBase<JacobianDerived>& jacobian_)
    {
        // Get a non-const reference, hack for Eigen to allow matrix.block() inputs. See:
        // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
        // Eigen::MatrixBase<StateDerived>& state = const_cast< Eigen::MatrixBase<StateDerived>& >(state_);
        Eigen::MatrixBase<JacobianDerived>& jacobian = const_cast< Eigen::MatrixBase<JacobianDerived>& >(jacobian_);
        
        // Form the noise jacobian, left to right, top to bottom
        // ORIENTATION ROW
        jacobian.template block<3,3>(ORIENTATION,GYRONOISE) = -mCbn;
        
        // POSITION ROW
        jacobian(PNORTH,FORWARDNOISE) = std::cos(mYaw);
        jacobian(PEAST,FORWARDNOISE) = std::sin(mYaw);
        jacobian(PNORTH,LATERALNOISE) = -std::sin(mYaw);
        jacobian(PEAST,LATERALNOISE) = std::cos(mYaw);
    }
    
    /*********************************************************************************//**
    * cInsErrorModel::operator()(cData<cInsData>&)
    * \return \c void
    * Orientation is required in jacobian calculations and is set via this callback
    * Template function cannot be specialized independently of template class so this
    * function is repeated for NINE_STATE, FIFTEEN_STATE specializations etc.
    **********************************************************************************/
    template<>
    void cPdrErrorModel::operator()(cMessage<cInsMsg>& data)
    {
        mYaw = data.derived().Orientation(YAW);
        BASIL::cINS::euler2cbn(data.derived().Orientation,mCbn);
    }
}

#endif // CPDRERRORMODEL_H
