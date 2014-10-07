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
 * @file   Ins.java
 * @brief  INS activity for quering IMU sensor and passing to the BASIL library
 * @author M. George
 */

package nrec.basil.androidins;

import android.content.res.AssetManager;

// Inertial navigation system class.  This is mostly a wrapper for the C++ version
// from the BASIL implementation.
public class Ins 
{
	// Call the native initialization function which will create the necessary
	// components and initialize them with parameters from the config-sage.xml
	// file in the assets.
	public native boolean init(AssetManager asset_manager);
	public native void close();
	
	// Send IMU data in for integration.  imudata = time,ax,ay,az,gx,gy,gz.
	public native void integrate(double[] imudata);
	
	// Poll INS for pose data, pose = time,r,p,y,x,y,z,vx,vy,vz
	public native double[] get_pose();
}