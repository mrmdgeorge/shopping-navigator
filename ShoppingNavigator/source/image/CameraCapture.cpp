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
 * @file   CameraCapture.cpp
 * @brief  An image capture class for Pt. Grey cameras
 * @author M. George
 */

#include <iostream>
#include <stdio.h>

#include "CameraCapture.h"

CameraCapture::CameraCapture()
{
    captureCount = 0;
    savedCount = 0;
    fileType = ".jpg";
    
    //Getting the GUID of the cam
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }
    
    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }
    
    //Starting the capture
    error = cam.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }
    
    //Get one raw image to be able to calculate the OpenCV window size
    cam.RetrieveBuffer(&rawImage);
    
    // Get the raw image dimensions
    rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
    
    /* create a window for the video */
    cvNamedWindow( "FireFly MV", CV_WINDOW_AUTOSIZE );
    
    //Setting the window size in OpenCV
    frame = cvCreateImage(cvSize(cols, rows), 8, 1);
}

CameraCapture::~CameraCapture()
{
    // Release the capture device housekeeping
    cvDestroyWindow( "mywindow" );
    cvReleaseImage(&frame);
    
    // Stop capturing images
    error = cam.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }      
    
    // Disconnect the camera
    error = cam.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }
}

// A Simple Camera Capture Framework 
bool CameraCapture::getFrame() 
{
    // Start capturing images
    cam.RetrieveBuffer(&rawImage);

    // Convert the raw image
    error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }

    //Copy the image into the IplImage of OpenCV
    memcpy(frame->imageData, convertedImage.GetData(), convertedImage.GetDataSize());

    // Get one frame
    cvShowImage( "FireFly MV", frame );
    captureCount++;
    //char key = cvWaitKey(10);
    cvWaitKey(5);
    if( captureCount % 30 == 0 )
    {
        std::cout << "Captured Image: " << savedCount + mapSize << std::endl;
        if ( !frame ) 
        {
            std::cerr << "ERROR: capture is NULL" << std::endl;
            getchar();
        }
        sprintf(captureFileName,"%.6u",savedCount + mapSize);
        cvSaveImage( (std::string(captureFileName)+fileType).c_str(), frame);
        savedCount++;
        return true;
    }
    else
    {
        return false;
    }
}

void CameraCapture::setMapSize(int size)
{
    mapSize = size;
}
