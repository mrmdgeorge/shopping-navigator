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
 * @file   CameraActivity.java
 * @brief  Activity to periodically grab camera images and send them to laptop 
 * @author M. George
 */

package nrec.basil.wimuconsole;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;

import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Camera;
import android.hardware.Camera.Parameters;
import android.hardware.Camera.PictureCallback;
import android.text.format.Time;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

public class CameraActivity extends Activity implements SurfaceHolder.Callback{

    // Debugging
    private static final String TAG = "wImuConsoleCamera";
    private static final boolean D = true;

    // Camera settings
    private static Camera mCamera = null;

    private Bitmap preview = null;
    private SurfaceHolder mHolder = null;
    private SurfaceView mView = null;

    // Timer for synchronous image grabs
    private Timer mImageTimer = null;
    private long mImageCaptureDelay = 5000;

    // Basil header for saving camera data
    private BluetoothService mBluetoothService = null;
    byte[] mBasilHeader;
    byte[] mBasilCrc;
    private Date mEpochTime;
    private ByteBuffer b;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        //get the Surface View at the main.xml file  
        mView = (SurfaceView) findViewById(R.id.surface_view);  

        //Get a surface  
        mHolder = mView.getHolder();  

        //add the callback interface methods defined below as the Surface View callbacks  
        mHolder.addCallback(this);

        mImageTimer = new Timer();

        mEpochTime = new Date();

        // Constant parts of the basil header.
        // Message size (bytes 8-9) and time info need to be set for each message
        mBasilHeader = new byte[22];
        b = ByteBuffer.wrap(mBasilHeader);
        mBasilHeader[0]  = 'C'; // Syncs
        mBasilHeader[1]  = 'M';
        mBasilHeader[2]  = 'U';
        mBasilHeader[3]  = 0x16; // Header size
        mBasilHeader[4]  = 0x00; // Hardware Id
        mBasilHeader[5]  = 0x0C; // Message type (Camera)
        mBasilHeader[6]  = 0x00; // Message version (NEXUS4_REAR)
        mBasilHeader[7]  = 0x00; // Device Id
        mBasilHeader[10] = 0x00; // Reserved
        mBasilHeader[11] = 0x03; // Time status (GPS Disciplined)
        short gpsWeek = (short)Math.floor((mEpochTime.getTime() - 315964800)/604800); // GPS Week number
        b.putShort(12, gpsWeek);

        // CRC initialized to zero by Java
        mBasilCrc = new byte[4];

        mBluetoothService = SensorSettingsActivity.mBluetoothService;
    }

    @Override
    protected void onResume() {
        super.onResume();
        mImageTimer.schedule(mCaptureTask, 1000, mImageCaptureDelay);
    }

    @Override
    protected void onStop(){
        super.onStop();
        mCamera.release();
        mCamera = null;
    }

    @Override
    protected void onPause() {
        super.onPause();
        //mImageTimer.cancel();
    }

    TimerTask mCaptureTask = new TimerTask(){
        public void run(){
            Log.d(TAG,"Camera Callback");
            mCamera.startPreview();
            mCamera.takePicture(null,null,mPicture);
        }
    };

    //sets what code should be executed after the picture is taken  
    Camera.PictureCallback mPicture = new Camera.PictureCallback()  
    {  
        @Override  
        public void onPictureTaken(byte[] data, Camera camera)  
        {  
            ImageView i;
            i = (ImageView)findViewById(R.id.camera_view);

            // Decode the data obtained by the camera into a Bitmap  
            preview = BitmapFactory.decodeByteArray(data, 0, data.length);  
            // Set the iv_image  
            i.setImageBitmap(preview);

            File pictureFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + Environment.DIRECTORY_DCIM,"testimage.basil");

            // Put GPS Week
            mEpochTime = new Date();
            long gpsMilliSeconds = mEpochTime.getTime();
            double gpsEpochSeconds = (double)gpsMilliSeconds/1000.0;
            short gpsWeek = (short)Math.floor((gpsEpochSeconds - 315964800)/604800); // GPS Week number
            b.putShort(12, gpsWeek);

            // Put GPS seconds
            double gpsDecimalSecondsSinceWeek = (gpsEpochSeconds - 315964800.0)%604800.0;
            Log.d(TAG,"GPS Int Time: " + gpsDecimalSecondsSinceWeek);
            int gpsIntSecondsSinceWeek = (int)Math.floor(gpsDecimalSecondsSinceWeek);
            Log.d(TAG,"GPS Nano Time: " + gpsIntSecondsSinceWeek);
            int gpsNanoSecondsSinceSecond = (int)((gpsDecimalSecondsSinceWeek - (double)gpsIntSecondsSinceWeek)*1e9);
            Log.d(TAG,"GPS Nano Time: " + gpsNanoSecondsSinceSecond);
            b.putInt(14, gpsIntSecondsSinceWeek);
            b.putInt(18, gpsNanoSecondsSinceSecond);

            // Put the size of the image in the header
            if (data.length > 65535){
                // Make use of the reserved byte and puts a 3 byte int in the log.  Good
                // for images up to ~16MB in size.  Two byte int tops out at 65K which works
                // for some small images like 640x480 .jpg's with little texture.
                byte[] customLength = ByteBuffer.allocate(4).putInt(data.length).array();
                b.put(8, customLength[1]);
                b.put(9, customLength[2]);
                b.put(10, customLength[3]);
            }else{
                b.putShort(8, (short)data.length);
            }

            if (mBluetoothService != null) {
                mBluetoothService.write(mBasilHeader);
                mBluetoothService.write(data);
                mBluetoothService.write(mBasilCrc);
                Log.d(TAG,"Writing data to bluetooth log: " + data.length);
            } else {
                mBluetoothService = SensorSettingsActivity.mBluetoothService;
                Log.d(TAG,"Failed writing data to bluetooth log.");
            }

            //            try {
            //                FileOutputStream fos = new FileOutputStream(pictureFile);
            //                fos.write(mBasilHeader);
            //                fos.write(data);
            //                fos.write(mBasilCrc);
            //                fos.close();
            //            } catch (FileNotFoundException e) {
            //                Log.d(TAG, "File not found: " + e.getMessage());
            //            } catch (IOException e) {
            //                Log.d(TAG, "Error accessing file: " + e.getMessage());
            //            }
        }  
    };  

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.camera, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.actionbar_sensor_settings:
            startActivity(new Intent(this, SensorSettingsActivity.class));
            break;
        case R.id.actionbar_map_view:
            startActivity(new Intent(this, MapActivity.class));
            break;
        default:
            break;
        }

        return true;
    }

    // Change the frame delay
    public void setFrameDelay(View view){
        //        // Check that we're actually connected before trying anything
        //        EditText frame_delay_input = (EditText)findViewById(R.id.camera_frame_delay);
        //        String frame_delay = frame_delay_input.getText().toString();
        //        mCaptureTask.cancel();
        //        mImageTimer.cancel();
        //        mImageTimer = new Timer();
        //        mImageTimer.schedule(mCaptureTask, 0, Long.parseLong(frame_delay));
        //        mCaptureTask = new TimerTask(){
        //            public void run(){
        //                //mCamera.startPreview();
        //                mCamera.takePicture(null,null,mPicture);
        //            }
        //        };
    }

    @Override
    public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3)  
    {  
        //get camera parameters  
        Parameters parameters = mCamera.getParameters();  

        //set camera parameters  
        mCamera.setParameters(parameters);  
        mCamera.startPreview();
    }  

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, acquire the camera and tell it where  
        // to draw the preview.  
        try {
            mCamera = Camera.open(); // attempt to get a Camera instance
            Log.d(TAG,"Get camera.");
        }
        catch (Exception e){
            Log.e(TAG,"Could not get camera.",e);
        }
        try {  
            mCamera.setPreviewDisplay(holder);  

        } catch (IOException exception) {  
            mCamera.release();  
            mCamera = null;  
        }  

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        // Stop the preview  
        mCamera.stopPreview();  
        // Release the camera  
        mCamera.release();  
        // Unbind the camera from this object  
        mCamera = null;  
    }
}
