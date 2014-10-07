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
 * @file   InertialTracker
 * @brief  Main file for pedestrian navigation w/ IMUs and cameras
 * @author M. George
 */

#include <iostream>
#include <csignal>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "basil/interfaces/cipsocket.h"
#include "basil/interfaces/cipserver.h"
#include "basil/interfaces/clogreader.h"
#include "basil/interfaces/clogger.h"
#include "basil/inertial/cins.h"
#include "basil/filters/ckalmanfilter.h"
#include "basil/messages/sensors/cimumsg.h"
#include "basil/messages/sensors/caxisswap.h"
#include "basil/messages/sensors/cinsmsg.h"
#include "basil/messages/internal/cinserrormsg.h"
#include "basil/inertial/cinserrormodel.h"

#include "cinertialfilter.h"
#include "cxsensusbport.h"

// Some things that the OS signal handler needs to see for shutdown on ctrl-c
void signal_handler(int);

// Some functions that will make main() easier to read
std::string appendDateTime(std::string logfilename);

int main(int argc, char* argv[])
{
    //******************************
    //
    // SETUP
    //
    //******************************
    if (argc == 1)
    {
        std::cout << "Error: You must specify a parameter file." << std::endl;
        return 1;
    }
    
    // Load in xml configuration file
    boost::property_tree::ptree config;
    boost::property_tree::read_xml(argv[1], config, boost::property_tree::xml_parser::trim_whitespace);

    // Log files will use the format from the config file appended with system time
    std::string datalogfilename = config.get<std::string>("parameters.logger.filename");
    datalogfilename = appendDateTime(datalogfilename);
    std::string errorlogfilename = datalogfilename + ".log";
    datalogfilename += std::string(".dat");
    
    // Handle the input switches.  Parameter files are not optional and therefore do not require a switch.
    char* datasource;
    bool runLive = true;
    if(argc >= 3)
    {
        for (int i = 2; i < argc; i++)
        {
            if (std::string(argv[i]) == "-logplayback")
            {
                // Get log filename from command line and turn off some live only settings, regardless of .xml values.
                datasource = argv[i+1];
                runLive = false;
                // Keep the same filename as the input if playing back a file
                datalogfilename = std::string(datasource);
                datalogfilename.erase(datalogfilename.end() - 4, datalogfilename.end());
                errorlogfilename = datalogfilename + ".log";
                datalogfilename += ".processed";
                i++;
            }
            else if (std::string(argv[i]) == "-logfilename")
            {
                // Change the output file name from it's default timestamped value.
                datalogfilename = std::string(argv[i+1]);
                i++;
            }
            else
            {
                std::cout << "IntelPose Error: Invalid option." << std::endl;
                return 1;
            }
        }
    }
    else
    {
        // We are running from live data with default logfilename
        std::string dummy("NULL");
        datasource = const_cast<char*>(dummy.c_str());
    }
    
    // Store the input file name if using logged data or "NULL" is using live data.
    std::string inputfile(datasource);
    
    // Redirect cerr to a file
    stderr = freopen(errorlogfilename.c_str(),"w",stderr);
    
    // Handle signals from the OS
    signal(SIGINT,signal_handler);
    
    //*********************************
    //
    // Load components and subscribe
    //
    //*********************************

    // We want to reorder the IMU axes
    BASIL::RotationMatrix R; R << 0.0,-1.0,0.0,0.0,0.0,1.0,-1.0,0.0,0.0;
    BASIL::cAxisSwap axisSwap(R);

    // Connect IMU data to the axis swapping object before anything else
    BASIL::cImuMsg::subscribe(boost::ref(axisSwap));

    // Inertial navigation systems for each shoe
    BASIL::cINS ins(config,"boot-ins");
    
    // State models for full inertial navigation
    //BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> state(config,"pose-filter");
    BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE> state(config,"pose-filter");

    // Single filter for camera aided navigation
    //cInertialFilter< BASIL::cKalmanFilter, BASIL::cInsErrorModel<BASIL::INS::NINE_STATE> > filter(state,config,"pose-filter");
    cInertialFilter< BASIL::cKalmanFilter, BASIL::cInsErrorModel<BASIL::INS::FIFTEEN_STATE> > filter(state,config,"pose-filter");
    
    // A binary logger to record experiments
    BASIL::cLogger logger(config, datalogfilename);
    
    // A UDP broadcast component to send INS data to the network
    BASIL::cIpServer broadcaster(config, "192.168.1.128", 54321);

    BASIL::cInsMsg::subscribe(boost::ref(broadcaster));
    BASIL::cPlaceRecognitionMsg::subscribe(boost::ref(broadcaster));
    
    // Connect data to subscribers
    BASIL::cImuMsg::subscribe(boost::ref(ins));

    // The state model needs Imu and Ins data for prediction
    BASIL::cImuMsg::subscribe(boost::ref(state));
    BASIL::cInsMsg::subscribe(boost::ref(state));
    
    // Filter needs Imu, Ins, Srs and Landmark data for prediction and correction
    BASIL::cImuMsg::subscribe(boost::ref(filter));
    BASIL::cInsMsg::subscribe(boost::ref(filter));
    BASIL::cPlaceRecognitionMsg::subscribe(boost::ref(filter));
    
    // The filter outputs will be errors that are returned to the INS for calibration
    BASIL::cInsErrorMsg::subscribe(boost::ref(ins));

    if (runLive)
    {
        BASIL::cImuMsg::subscribe(boost::ref(logger));
        BASIL::cPlaceRecognitionMsg::subscribe(boost::ref(logger));
        BASIL::cInsMsg::subscribe(boost::ref(logger));
    }
    else
    {
        BASIL::cInsMsg::subscribe(boost::ref(logger));
        BASIL::cImuCalibrationMsg::subscribe(boost::ref(logger));
    }
    
    // Start the necessary interface
    if(runLive)
    {
        //BASIL::cXsens leftXsens(std::string("/dev/ttyUSB0"),115200,0);
        cXsensUsbPort<BASIL::cBasilInterface> xsens;
        // One camera communicating from a different process on the local host
        BASIL::cIpSocket<BASIL::cBasilInterface> camera(config,"camera");
        
        // Create and start each thread, data subscribers should handle mutexing 
        // if receiving data originating from different threads.
        boost::thread xsensThread(boost::bind(&cXsensUsbPort<BASIL::cBasilInterface>::connect,boost::ref(xsens)));
        boost::thread cameraThread(boost::bind(&BASIL::cIpSocket<BASIL::cBasilInterface>::connect,boost::ref(camera)));
        xsensThread.join();
        cameraThread.join();
    }
    else
    {
        // If we're not live we simply have a log reader
        BASIL::cLogReader<BASIL::cBasilInterface> logreader(datasource);
        logreader.connect();
    }
}

// Handle Operating System signals here.
void signal_handler(int signal)
{
    // Stop the sensor and cleanly finish logging data before shutting down.
    std::cout << "EXITING" << std::endl;
    exit(signal);
}

// Append the current system time to logfile names.
std::string appendDateTime(std::string logfilename)
{
    time_t rawtime;
    struct tm* timeinfo;
    std::ostringstream logfileh;
    std::ostringstream logfilem;
    std::ostringstream logfiles;
    std::ostringstream logfiley;
    std::ostringstream logfilemon;
    std::ostringstream logfiled;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    logfileh << "-";
    logfileh.fill('0');
    logfileh.width(2);
    logfilem << ".";
    logfilem.fill('0');
    logfilem.width(2);
    logfiles << ".";
    logfiles.fill('0');
    logfiles.width(2);
    logfiley << "-";
    logfiley.width(4);
    logfilemon << ".";
    logfilemon.fill('0');
    logfilemon.width(2);
    logfiled << ".";
    logfiled.fill('0');
    logfiled.width(2);
    logfiley << (timeinfo->tm_year + 1900);
    logfilemon << std::right << (timeinfo->tm_mon + 1);
    logfiled << std::right << timeinfo->tm_mday;
    logfileh << std::right << timeinfo->tm_hour; 
    logfilem << std::right << timeinfo->tm_min;
    logfiles << std::right << timeinfo->tm_sec;
    logfilename += logfiley.str() + logfilemon.str() + logfiled.str() + logfileh.str() + logfilem.str() + logfiles.str();
    return logfilename;
}
