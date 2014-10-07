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
 * @file   CameraCapture.h
 * @brief  An image capture class for Pt. Grey cameras
 * @author M. George
 */
#ifndef CAMERACAPTURE_H
#define CAMERACAPTURE_H

#include <string>

#include <FlyCapture2.h>

#include <cv.h>
#include <highgui.h> 

class CameraCapture
{
public:
    CameraCapture();
    ~CameraCapture();
    bool getFrame();
    void setMapSize(int);
    unsigned int captureCount;
    unsigned int savedCount;
    char captureFileName[10];
    std::string fileType;
    int mapSize;

    // OpenCV data
    IplImage* frame;

    // Pt. Grey flycap data
    FlyCapture2::Error error;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::BusManager busMgr;
    FlyCapture2::Camera cam;
    FlyCapture2::Image rawImage;
    FlyCapture2::Image convertedImage;
    FlyCapture2::PixelFormat pixFormat;
    unsigned int rows;
    unsigned int cols;
    unsigned int stride;
};

#endif //CAMERACAPTURE_H
