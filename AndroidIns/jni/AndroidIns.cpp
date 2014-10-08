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
 * @file   AndroidIns.cpp
 * @brief  JNI calls to the BASIL INS library
 * @author M. George
 */

#include "nrec_basil_androidins_Ins.h"

#include <android/asset_manager_jni.h>
#include <android/log.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>

#include <basil/basil.h>
#include <basil/inertial/cins.h>
#include <basil/inertial/cinserrormodel.h>
#include <basil/messages/sensors/cimumsg.h>
#include <basil/messages/internal/cinserrormsg.h>
#include <basil/filters/measurements/czuptdetector.h>

#include "csagefilter.h"

#ifdef __cplusplus
extern "C" {
#endif

boost::property_tree::ptree* config;
BASIL::cINS* ins;
BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>* error_model;
SAGE::cSageFilter<BASIL::cKalmanFilter,
BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> >* filter;
double sensor_time;
BASIL::cImuMsg* imu_msg;

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    init
 * Signature: ()V
 */
JNIEXPORT jboolean JNICALL Java_nrec_basil_androidins_Ins_init(JNIEnv* env, jobject thiz, jobject asset_manager)
{
    __android_log_print(ANDROID_LOG_INFO, "AndroidIns", "Initializing ...");

    sensor_time = 0.0;

    // Setup the static components of an IMU message
    imu_msg = new BASIL::cImuMsg;
    imu_msg->MessageType = BASIL::MESSAGE::IMU;
    imu_msg->MessageVersion = BASIL::IMU::BASIL_IMU;
    imu_msg->HardwareId = 0;
    imu_msg->DeviceId = 0;
    imu_msg->Time.setTime(BASIL::TIME::FREE_RUNNING, 0, sensor_time);
    imu_msg->ImuMdl = BASIL::IMU::BASIL_IMU;
    imu_msg->ImuStatus = BASIL::IMU::GOOD;
    imu_msg->DataType = BASIL::IMU::RATE;
    imu_msg->Acceleration.setZero();
    imu_msg->AngularRate.setZero();
    imu_msg->DeltaVelocity.setZero();
    imu_msg->DeltaAngle.setZero();
    imu_msg->Temperature = 0.0;

    // Get the configuration file from the package assets
    AAssetManager* mgr = AAssetManager_fromJava(env, asset_manager);
    AAsset* asset = AAssetManager_open(mgr, "config-sage.xml",AASSET_MODE_BUFFER);
    // Check we were able to get something
    if (asset == NULL) {
        __android_log_print(ANDROID_LOG_INFO, "AndroidIns",
                "Could not open asset ...");
        return JNI_FALSE;
    }
    // Copy the asset into a local buffer
    long size = AAsset_getLength(asset);
    char* config_file = new char[size];
    int copied = AAsset_read(asset, config_file, size);

    // Populate the property tree with the data, read_xml needs a filename or an istream so we use an istringstream
    config = new boost::property_tree::ptree;
    std::istringstream config_stream(std::string(config_file, copied));
    boost::property_tree::read_xml(config_stream, *config,boost::property_tree::xml_parser::trim_whitespace);

    // Close the asset and delete the buffer since we now have a populate ptree
    AAsset_close(asset);
    delete[] config_file;

    // Initialize the INS
    ins = new BASIL::cINS(*config, "sage-main");
    error_model = new BASIL::cInsErrorModel<BASIL::INS::NINE_STATE>(*config,"sage-main");
    filter = new SAGE::cSageFilter<BASIL::cKalmanFilter,BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> >(*error_model,*config, "sage-main");

    // Make subscription connections
    BASIL::cImuMsg::subscribe(boost::ref(*ins));
    BASIL::cImuMsg::subscribe(boost::ref(*error_model));
    BASIL::cImuMsg::subscribe(boost::ref(*filter));
    BASIL::cInsMsg::subscribe(boost::ref(*error_model));
    BASIL::cInsMsg::subscribe(boost::ref(*filter));
    BASIL::cInsErrorMsg::subscribe(boost::ref(*ins));

    return JNI_TRUE;
}

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_nrec_basil_androidins_Ins_close(JNIEnv* env, jobject thiz)
{
    // Delete our objects in reverse order
    delete filter;
    delete error_model;
    delete ins;
    delete config;
    delete imu_msg;
}

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    integrate
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_nrec_basil_androidins_Ins_integrate(JNIEnv* env, jobject thiz, jdoubleArray imudata)
{
    // Update the imu_msg fields with the new data
    env->GetDoubleArrayRegion(imudata, 0, 1, &sensor_time);
    imu_msg->Time.setTime(BASIL::TIME::FREE_RUNNING, 0, sensor_time);
    env->GetDoubleArrayRegion(imudata, 1, 3, imu_msg->Acceleration.data());
    env->GetDoubleArrayRegion(imudata, 4, 3, imu_msg->AngularRate.data());

    // Call IMU subscribers
    BASIL::cImuMsg::notify(*imu_msg);
}

/*
 * Class:     nrec_basil_androidins_Ins
 * Method:    get_pose
 * Signature: ()[D
 */
JNIEXPORT jdoubleArray JNICALL Java_nrec_basil_androidins_Ins_get_1pose(JNIEnv* env, jobject thiz) {
    // Create a Java array to carry our pose output
    jdoubleArray pose = env->NewDoubleArray(10);

    // Check we were able to get memory for it
    if (pose == NULL) {
        return NULL;
    }

    sensor_time = ins->mInsData.Time.getSeconds();
    env->SetDoubleArrayRegion(pose, 0, 1, &sensor_time);
    env->SetDoubleArrayRegion(pose, 1, 3, ins->mInsData.Orientation.data());
    env->SetDoubleArrayRegion(pose, 4, 3, ins->mInsData.Position.data());
    //__android_log_print(ANDROID_LOG_INFO, "AndroidIns", "Eulers: %lf, %lf, %lf", ins->mInsData.Position(0), ins->mInsData.Position(1), ins->mInsData.Position(2));
    env->SetDoubleArrayRegion(pose, 7, 3, ins->mInsData.Velocity.data());

    return pose;
}

#ifdef __cplusplus
}
#endif
