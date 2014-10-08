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
 * @file   MainActivity.java
 * @brief  Main file for an inertial navigation system using built-in device IMU
 * @author M. George
 */

package nrec.basil.androidins;

import java.text.DecimalFormat;

import android.os.Bundle;
import android.app.Activity;
import android.content.res.AssetManager;
import android.util.Log;
import android.view.Menu;
import android.widget.TextView;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class MainActivity extends Activity implements SensorEventListener {
    private SensorManager sensorManager;
    // Access to our configuration .xml file happens through an asset manager
    private AssetManager assetManager;
    private Ins ins = new Ins();
    private boolean gotAccelerometer = false;
    private SensorEvent accelerometerData;
    private boolean gotGyroscope = false;
    private SensorEvent gyroData;
    private boolean ins_up = false;
    private static final double NS2S = 1.0d / 1000000000.0d;
    private static final double R2D = 180.0d / 3.14159d;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        assetManager = (AssetManager) getAssets();
        ins_up = ins.init(assetManager);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            accelerometerData = event;
            gotAccelerometer = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            gyroData = event;
            gotGyroscope = true;
        }
        if (gotAccelerometer && gotGyroscope) {
            if (ins_up) {
                sendImuData(accelerometerData, gyroData);
            }
        }
    }

    private void sendImuData(SensorEvent accelerometer, SensorEvent gyro) {
        double[] imuData = new double[7];
        double[] insData;

        imuData[0] = accelerometer.timestamp * NS2S;
        imuData[1] = accelerometer.values[0];
        imuData[2] = accelerometer.values[1];
        imuData[3] = accelerometer.values[2];
        imuData[4] = gyro.values[0];
        imuData[5] = gyro.values[1];
        imuData[6] = gyro.values[2];

        // Send the IMU data
        ins.integrate(imuData);

        // Receive the INS result
        insData = ins.get_pose();

        // Display the data
        displayData(imuData, insData);

        // Reset the flags for next acquisition
        gotAccelerometer = gotGyroscope = false;
    }

    private void displayData(double[] imu, double[] ins) {
        TextView t;
        DecimalFormat form = new DecimalFormat("###.#");

        // Display IMU data
        t = (TextView) findViewById(R.id.ax);
        t.setText(form.format(imu[1]));
        t = (TextView) findViewById(R.id.ay);
        t.setText(form.format(imu[2]));
        t = (TextView) findViewById(R.id.az);
        t.setText(form.format(imu[3]));
        t = (TextView) findViewById(R.id.gx);
        t.setText(form.format(imu[4] * R2D));
        t = (TextView) findViewById(R.id.gy);
        t.setText(form.format(imu[5] * R2D));
        t = (TextView) findViewById(R.id.gz);
        t.setText(form.format(imu[6] * R2D));

        Log.v("ImuData",
                Double.toString(imu[0]) + " " + Double.toString(imu[1]) + " "
                        + Double.toString(imu[2]) + " "
                        + Double.toString(imu[3]) + " "
                        + Double.toString(imu[4]) + " "
                        + Double.toString(imu[5]) + " "
                        + Double.toString(imu[6]));

        // Display INS data
        // Log.i("JavaAndroidIns","Time (J): " + Double.toString(ins[0]));
        t = (TextView) findViewById(R.id.ox);
        t.setText(form.format(ins[1] * R2D));
        t = (TextView) findViewById(R.id.oy);
        t.setText(form.format(ins[2] * R2D));
        t = (TextView) findViewById(R.id.oz);
        t.setText(form.format(ins[3] * R2D));
        t = (TextView) findViewById(R.id.px);
        t.setText(form.format(ins[4]));
        t = (TextView) findViewById(R.id.py);
        t.setText(form.format(ins[5]));
        t = (TextView) findViewById(R.id.pz);
        t.setText(form.format(ins[6]));
        t = (TextView) findViewById(R.id.vx);
        t.setText(form.format(ins[7]));
        t = (TextView) findViewById(R.id.vy);
        t.setText(form.format(ins[8]));
        t = (TextView) findViewById(R.id.vz);
        t.setText(form.format(ins[9]));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    protected void onResume() {
        super.onResume();
        // register this class as a listener for the orientation and
        // accelerometer sensors
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPause() {
        // unregister listener
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    static {
        System.loadLibrary("AndroidIns");
    }
}
