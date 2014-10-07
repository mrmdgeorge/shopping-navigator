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
 * @file   SensorSettingsActivity.java
 * @brief  Activity to modify bluetooth IMU settings
 * @author M. George
 */
package nrec.basil.wimuconsole;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.util.Arrays;

import nrec.basil.wimuconsole.BluetoothService;
import nrec.basil.wimuconsole.R;

import android.app.ActionBar;
import android.app.Activity;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentActivity;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;

public class SensorSettingsActivity extends FragmentActivity implements
    ActionBar.OnNavigationListener {
    /***************************************/
    /***    APPLICATION LEVEL SETTINGS   ***/
    /***************************************/
    private static final String TAG = "wImuConsole-SensorSettings";
    private static final boolean D = true;
    // Key for saving and restoring the action bar dropdown
    private static final String STATE_SELECTED_NAVIGATION_ITEM = "selected_navigation_item";
    
    /***************************************/
    /***        BLUETOOTH SETTINGS       ***/
    /***************************************/    
    // Bluetooth adapter, services and settings
    private BluetoothAdapter mBluetoothAdapter = null;
    public static BluetoothService mBluetoothService = null;    
    private String mConnectedDeviceName = null;
    
    // Key values for bluetooth message passing
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";
    
    // Bluetooth thread handler message IDs
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;
    
    // List of IMUs that can be connected to
    Spinner imuSpinner;
    private String wImuMacAddress = null;    
   
    // Intent request codes
    private static final int REQUEST_ENABLE_BT = 3;
    
    /***************************************/
    /***         BASIL SETTINGS          ***/
    /***************************************/
    private static final byte[] basilStart = new byte[26];
    private static final byte[] basilStop = new byte[26];
    private static final byte[] basilReset = new byte[26];
    
    public static double mCurrentLatitude = 0.0;
    public static double mCurrentLongitude = 0.0;
    
    public boolean mLoggingOn = false;
    public String mLogFileName = null;
    
    /***************************************/
    /***            CONSTANTS            ***/
    /***************************************/
    private static final double R2D = 180.0/3.141519;
    private static final double D2R = 1.0/R2D;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_settings);

        // Set up the action bar to show a dropdown list which will
        // contain the various sensors that can be configured.  Some
        // sensors are local (camera) some are remote bluetooth devices
        // (IMU, Barometer etc.)
        final ActionBar actionBar = getActionBar();
        actionBar.setDisplayShowTitleEnabled(false);
        actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_LIST);

        // Set up the dropdown list navigation in the action bar.
        actionBar.setListNavigationCallbacks(
        // Specify a SpinnerAdapter to populate the dropdown list.
        new ArrayAdapter<String>(actionBar.getThemedContext(),
                android.R.layout.simple_list_item_1,
                android.R.id.text1, getResources().getStringArray(R.array.sensors_list) ), this);
        
        // Specify a SpinnerAdapter to populate the wIMU list
        ArrayAdapter<CharSequence> imuAdapter = ArrayAdapter.createFromResource(this, R.array.wimu_list, android.R.layout.simple_spinner_item);
        imuAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        imuSpinner = (Spinner) findViewById(R.id.wimulist);
        imuSpinner.setAdapter(imuAdapter);
        
        // Load the BASIL start/stop/reset commands
        InputStream input = getResources().openRawResource(R.raw.basilstart);
        try {
            input.read(basilStart,0,26);
        } catch (IOException e) {
            Log.e(TAG,"Could not read BASIL start command",e);
        }
        input = getResources().openRawResource(R.raw.basilstop);
        try {
            input.read(basilStop,0,26);
        } catch (IOException e) {
            Log.e(TAG,"Could not read BASIL stop command",e);
        }
        input = getResources().openRawResource(R.raw.basilreset);
        try {
            input.read(basilReset,0,26);
        } catch (IOException e) {
            Log.e(TAG,"Could not read BASIL reset command",e);
        }
        
        // Get the default filename for logging
        EditText logfilename = (EditText)findViewById(R.id.logfilename);
        mLogFileName = logfilename.toString();
        
        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
    }

    @Override
    public void onStart() {
        super.onStart();
        if(D) Log.e(TAG, "++ ON START ++");
        // Nothing to do here for the moment
    }
    
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
        case REQUEST_ENABLE_BT:
            // When the request to enable Bluetooth returns
            if (resultCode == Activity.RESULT_OK) {
                mBluetoothService = new BluetoothService(this, mHandler);
            } else {
                // User did not enable Bluetooth or an error occurred
                Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }
    
    // Send a BASIL start message to the connected IMU
    public void connectToImu(View view){
        // If BT is not on, request that it be enabled.
        // setupChat() will then be called during onActivityResult
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
            return;
        } else {
            if (mBluetoothService == null){
                // Initialize the BluetoothChatService to perform bluetooth connections
                mBluetoothService = new BluetoothService(this, mHandler);
            }
        }
        
        // Get the IMU MAC address from our list
        String address = imuSpinner.getSelectedItem().toString();
        // Get the BluetoothDevice object
        BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
        // Attempt to connect to the device
        mBluetoothService.connect(device, false);
    }
    
    // Send a BASIL start message to the connected IMU
    public void sendBasilStartMsg(View view){
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

        // Check that there's actually something to send
        if (basilStart.length > 0) {
            // Get the message bytes and tell the BluetoothChatService to write
            mBluetoothService.write(basilStart);
        }
    }
    
    // Send a BASIL stop message to the connected IMU
    public void sendBasilStopMsg(View view){
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

        // Check that there's actually something to send
        if (basilStop.length > 0) {
            if (D) Log.d(TAG, "Sending BASIL Stop");
            // Get the message bytes and tell the BluetoothChatService to write
            mBluetoothService.write(basilStop);
        }
    }
    
    // Send a BASIL reset message to the connected IMU
    public void sendBasilResetMsg(View view){
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

        // Check that there's actually something to send
        if (basilReset.length > 0) {
            // Get the message bytes and tell the BluetoothChatService to write
            mBluetoothService.write(basilReset);
        }
    }
    
    // Start or stop logging data
    public void toggleLogging(View view){
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }
        if (mLoggingOn == false){           
            mLoggingOn = true;
            // Set the filename with current name field
            EditText logfilename = (EditText)findViewById(R.id.logfilename);
            mLogFileName = logfilename.getText().toString();
            
            mBluetoothService.startLogging(mLogFileName);
            
            // Change the button to "Stop"
            Button b = (Button)findViewById(R.id.logbutton);
            b.setText("Stop");
        }
        else{
            mLoggingOn = false;
            mBluetoothService.stopLogging();
            // Display INS data
            Button b = (Button)findViewById(R.id.logbutton);
            b.setText("Start");
        }
    }
    
    // The Handler that gets information back from the BluetoothChatService
    private final Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
            case MESSAGE_STATE_CHANGE:
                if(D) Log.i(TAG, "MESSAGE_STATE_CHANGE: " + msg.arg1);
                switch (msg.arg1) {
                case BluetoothService.STATE_CONNECTED:
                    //setStatus(getString(R.string.title_connected_to, mConnectedDeviceName));
                    //mConversationArrayAdapter.clear();
                    break;
                case BluetoothService.STATE_CONNECTING:
                    //setStatus(R.string.title_connecting);
                    break;
                case BluetoothService.STATE_LISTEN:
                case BluetoothService.STATE_NONE:
                    //setStatus(R.string.title_not_connected);
                    break;
                }
                break;
            case MESSAGE_WRITE:
                byte[] writeBuf = (byte[]) msg.obj;
                // construct a string from the buffer
                //String writeMessage = new String(writeBuf);
                //if (D) Log.d(TAG, writeMessage);
                //TextView t = (TextView)findViewById(R.id.bufferdump);
                //t.setText(writeMessage);
                //mConversationArrayAdapter.add("Me:  " + writeMessage);
                break;
            case MESSAGE_READ:
                byte[] readBuf = (byte[]) msg.obj;
                if (D) Log.d(TAG, "Message Buffer: " + msg.arg1);                
                byte[] ins = new byte[101];
                ins = Arrays.copyOf(readBuf,msg.arg1);
                // construct a string from the valid bytes in the buffer
                displayInsData(ins);
                //String readMessage = new String(readBuf, 0, msg.arg1);
                //TextView t = (TextView)findViewById(R.id.bufferdump);
                //t.setText(readMessage);
                //mConversationArrayAdapter.add(mConnectedDeviceName+":  " + readMessage);
                break;
            case MESSAGE_DEVICE_NAME:
                // save the connected device's name
                mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
                Toast.makeText(getApplicationContext(), "Connected to "
                               + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
                break;
            case MESSAGE_TOAST:
                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
                               Toast.LENGTH_SHORT).show();
                break;
            }
        }
    };
    
    private void displayInsData(byte[] ins) {
        TextView t;
        DecimalFormat form = new DecimalFormat("##0.00");
        ByteBuffer insbuffer = ByteBuffer.wrap(ins);
        insbuffer.order(ByteOrder.LITTLE_ENDIAN);
        // Display INS data
        t = (TextView)findViewById(R.id.statusreport);
        String status = null;
        byte stat = insbuffer.get(22);
        if (D) Log.d(TAG, "INS Status: " + stat);    
        switch(stat)
        {
        case 0:
           status = "Calibrating";
           break;
        case 1:
            status = "Aligning";
            break;
        case 2:
            status = "Navigating";
            break;
        case 3:
            status = "Invalid Data";
            break;
        case 4:
            status = "Stopped";
            break;
        }
        t.setText(status);

        t = (TextView)findViewById(R.id.solutionreport);
        String solution = null;
        solution = String.format("%8s", Integer.toBinaryString(insbuffer.get(23) & 0xFF)).replace(' ', '0');
        t.setText(solution);
        
        t = (TextView)findViewById(R.id.px);
        mCurrentLatitude = insbuffer.getDouble(25);
        t.setText(form.format(insbuffer.getDouble(25)));
        t = (TextView)findViewById(R.id.py);
        mCurrentLongitude = insbuffer.getDouble(33);
        t.setText(form.format(insbuffer.getDouble(33)));
        t = (TextView)findViewById(R.id.pz); 
        t.setText(form.format(insbuffer.getDouble(41)));
        t = (TextView)findViewById(R.id.vx); 
        t.setText(form.format(insbuffer.getDouble(49)));
        t = (TextView)findViewById(R.id.vy); 
        t.setText(form.format(insbuffer.getDouble(57)));
        t = (TextView)findViewById(R.id.vz); 
        t.setText(form.format(insbuffer.getDouble(65)));
        t = (TextView)findViewById(R.id.ox); 
        t.setText(form.format(insbuffer.getDouble(73) * R2D));
        t = (TextView)findViewById(R.id.oy); 
        t.setText(form.format(insbuffer.getDouble(81) * R2D));
        t = (TextView)findViewById(R.id.oz); 
        t.setText(form.format(insbuffer.getDouble(89) * R2D));
    }
    
    @Override
    public void onRestoreInstanceState(Bundle savedInstanceState) {
        // Restore the previously serialized current dropdown position.
        if (savedInstanceState.containsKey(STATE_SELECTED_NAVIGATION_ITEM)) {
            getActionBar().setSelectedNavigationItem(
                    savedInstanceState.getInt(STATE_SELECTED_NAVIGATION_ITEM));
        }
    }

    @Override
    public void onSaveInstanceState(Bundle outState) {
        // Serialize the current dropdown position.
        outState.putInt(STATE_SELECTED_NAVIGATION_ITEM, getActionBar()
                .getSelectedNavigationIndex());
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.sensor_settings, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
      switch (item.getItemId()) {
      case R.id.actionbar_map_view:
          startActivity(new Intent(this, MapActivity.class));
        break;
      case R.id.actionbar_camera_settings:
          startActivity(new Intent(this, CameraActivity.class));
        break;
      default:
        break;
      }

      return true;
    } 
    
    @Override
    public boolean onNavigationItemSelected(int position, long id) {
        // When the given dropdown item is selected, show its contents in the
        // container view.
        //if (position == 0){
        //    startActivity(new Intent(this, MainActivity.class));
        //}
        //else{
        //    startActivity(new Intent(this, MapActivity.class));
        //}
        //Fragment fragment = new DummySectionFragment();
        //Bundle args = new Bundle();
        //args.putInt(DummySectionFragment.ARG_SECTION_NUMBER, position + 1);
        //fragment.setArguments(args);
        //getSupportFragmentManager().beginTransaction()
        //        .replace(R.id.container, fragment).commit();
        return true;
    }

    /**
     * A dummy fragment representing a section of the app, but that simply
     * displays dummy text.
     */
    public static class DummySectionFragment extends Fragment {
        /**
         * The fragment argument representing the section number for this
         * fragment.
         */
        public static final String ARG_SECTION_NUMBER = "section_number";

        public DummySectionFragment() {
        }

        @Override
        public View onCreateView(LayoutInflater inflater, ViewGroup container,
                Bundle savedInstanceState) {
            View rootView = inflater.inflate(R.layout.fragment_ins_settings,
                    container, false);
            TextView dummyTextView = (TextView) rootView
                    .findViewById(R.id.section_label);
            dummyTextView.setText(Integer.toString(getArguments().getInt(
                    ARG_SECTION_NUMBER)));
            return rootView;
        }
    }

}
