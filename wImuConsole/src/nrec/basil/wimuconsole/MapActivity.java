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
 * @file   MapActivity.java
 * @brief  Activity to initialize and display position on a Google Map
 * @author M. George
 */
package nrec.basil.wimuconsole;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

import android.location.Location;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.app.ActionBar;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.LocationSource;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.GoogleMap.OnCameraChangeListener;
import com.google.android.gms.maps.GoogleMap.OnMapClickListener;
import com.google.android.gms.maps.GoogleMap.OnMapLongClickListener;
import com.google.android.gms.maps.LocationSource.OnLocationChangedListener;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import android.os.Bundle;
import android.support.v4.app.FragmentActivity;

public class MapActivity extends FragmentActivity implements
    OnMapLongClickListener, ActionBar.OnNavigationListener{

    // Debugging
    private static final String TAG = "wImuConsoleMap";
    private static final boolean D = true;
    
    private GoogleMap mMap;
    private Marker mInitialPosition;
    private Marker mCurrentPosition;
    private Polyline mPolyline = null;
    private PolylineOptions mPolylineOptions;
    private GroundOverlay mNRECOverlay;
    private GroundOverlay mIntelOverlay;
    
    private boolean mInitialPositionSet = false;
    Handler mUIHandler;
    
    private static final double R2D = 180.0/3.141519;
    private static final double D2R = 1.0/R2D;
    
    private TimerTask positionPlot = new TimerTask(){
        public void run(){
            plotInsData();
        }
    };
    private Timer timer = new Timer();
    //timer.schedule(positionPlot);
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map);
   
        setUpMapIfNeeded();
        
        mUIHandler = new Handler(Looper.getMainLooper());
        mPolylineOptions = new PolylineOptions();
        mPolylineOptions.width(5)
                        .color(Color.BLUE)
                        .geodesic(true);
        timer.schedule(positionPlot,2000,500);
    }

    @Override
    protected void onResume() {
        super.onResume();
        setUpMapIfNeeded();
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.map, menu);
        return true;
    }

    /**
     * Sets up the map if it is possible to do so (i.e., the Google Play services APK is correctly
     * installed) and the map has not already been instantiated.. This will ensure that we only ever
     * call {@link #setUpMap()} once when {@link #mMap} is not null.
     * <p>
     * If it isn't installed {@link SupportMapFragment} (and
     * {@link com.google.android.gms.maps.MapView MapView}) will show a prompt for the user to
     * install/update the Google Play services APK on their device.
     * <p>
     * A user can return to this FragmentActivity after following the prompt and correctly
     * installing/updating/enabling the Google Play services. Since the FragmentActivity may not have been
     * completely destroyed during this process (it is likely that it would only be stopped or
     * paused), {@link #onCreate(Bundle)} may not be called again so we should call this method in
     * {@link #onResume()} to guarantee that it will be called.
     */
    private void setUpMapIfNeeded() {
        // Do a null check to confirm that we have not already instantiated the map.
        if (mMap == null) {
            // Try to obtain the map from the SupportMapFragment.
            mMap = ((SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map))
                    .getMap();
            
            // Check if we were successful in obtaining the map.
            if (mMap != null) {
                setUpMap();
            }
        }
    }

    /**
     * This is where we can add markers or lines, add listeners or move the camera. In this case, we
     * just add a marker near Africa.
     * <p>
     * This should only be called once and when we are sure that {@link #mMap} is not null.
     */
    private void setUpMap() {
        mMap.setOnMapLongClickListener(this);
        mMap.setMyLocationEnabled(true);
        mInitialPosition = mMap.addMarker(new MarkerOptions()
                               .position(new LatLng(0,0))
                               .title("Initial Position")
                               .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED))
                               .visible(false));
        mNRECOverlay = mMap.addGroundOverlay(new GroundOverlayOptions()
                             .image(BitmapDescriptorFactory.fromResource(R.drawable.nrec_floorplan_notext)).anchor(222/3150, 1)
                             .position(new LatLng(40.472513,-79.965032), 119.76f, 58.63f)
                             .transparency(0.75f)
                             .bearing(-57.9f));
        
        mIntelOverlay = mMap.addGroundOverlay(new GroundOverlayOptions()
                            .image(BitmapDescriptorFactory.fromResource(R.drawable.intel_floorplan_notext)).anchor(1, 1)
                            .position(new LatLng(40.4436194,-79.94638056), 45.5f, 40.85f)
                            .transparency(0.25f)
                            .bearing(16.5f));
        //mInitialPosition.setVisible(false);
    }

    @Override
    public void onMapLongClick(LatLng point) {
        // Move the map so that it is centered on the mutable polyline.
        mMap.moveCamera(CameraUpdateFactory.newLatLng(point));
        
        // Uses a colored icon.
        mInitialPosition.setPosition(point);
        mInitialPosition.setVisible(true);
        mInitialPositionSet = true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
      switch (item.getItemId()) {
      case R.id.actionbar_sensor_settings:
          startActivity(new Intent(this, SensorSettingsActivity.class));
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
    protected void onPause() {
        super.onPause();
    }
    
    private void plotInsData() {
        
        //double latIncrement;
        //double lonIncrement;

        //if (D) Log.d(TAG, "Increments: " + latIncrement + ", " + lonIncrement);       
        if (mInitialPositionSet) {
            //double currentLatitude = (mInitialPosition.getPosition().latitude*D2R + latIncrement)*R2D;
            //double currentLongitude = (mInitialPosition.getPosition().longitude*D2R + lonIncrement)*R2D;
            
            new AsyncTask<String, String, String>() {
                protected void onPreExecute() {
                    // perhaps show a dialog 
                    // with a progress bar
                    // to let your users know
                    // something is happening
                }

                protected void onPostExecute(String aResult) {
                    // background work is finished, 
                    // we can update the UI here
                    // including removing the dialog
                    double[] res = new double[2];
                    res[0] = (mInitialPosition.getPosition().latitude*D2R + SensorSettingsActivity.mCurrentLatitude / 6378000.0)*R2D;
                    res[1] = (mInitialPosition.getPosition().longitude*D2R + SensorSettingsActivity.mCurrentLongitude / 6378000.0)*R2D;
                    
                    mPolylineOptions.add(new LatLng(res[0],res[1]));
                    if (mPolyline != null){
                        mPolyline.remove();
                    }
                    mPolyline = mMap.addPolyline(mPolylineOptions);
                    //Marker mCurrentPosition = mMap.addMarker(new MarkerOptions()
                    //                              .position(new LatLng(res[0],res[1]))
                    //                              .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE))
                    //                              .visible(true));
                }

                @Override
                protected String doInBackground(String... params) {
                    // TODO Auto-generated method stub
                    return "Done";
                }
            }.execute();

        }
    }

    @Override
    public boolean onNavigationItemSelected(int itemPosition, long itemId) {
        // TODO Auto-generated method stub
        return false;
    }
    
}
