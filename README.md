1. ACKNOWLEDGEMENTS
-------------------

The software contained herein was developed by the National Robotics Engineering Center (NREC) at Carnegie Mellon University (CMU) under funding from the Intel Science and Technology Center for Embedded Computing (ISTC-EC) as part of the Personal Navigation for New Shopping Experiences project.  Contributors include Michael George, Michel Laverne, Dane Bennington, Prof. Alonzo Kelly & Prof. Tamal Mukherjee.

2. PURPOSE
----------

The Personal Navigation for New Shopping Experiences project developed and demonstrated proof-of-concept systems enabling indoor and GPS free navigation techniques for pedestrians.  Several combinations of sensors (IMUs, cameras and custom radar) were evaluated.  The software contained herein allows indoor navigation based on an IMU and a camera.

3. LIMITATIONS
--------------

This is proof-of-concept work.  It has been developed to the stage of initial demonstration with prototype hardware.  This software is only one component of a complete personal navigation system.  You will require hardware (IMU, camera, Android cellphone etc.) and third party software to complete a system.

4. COMPONENTS
-------------

* *ShoppingNavigator* - A linux (or Mac OS X) software package that uses an Xsens IMU and a Pt. Grey camera to perform pedestrian inertial navigation or pedestrian dead-reckoning (Step counting) w/ corrections from a pre-built map and scene recognition provided by the Oxford FABMAP system.

* *wImuConsole* - An Android app. that interfaces to the ShoppingNavigator components but uses a custom wireless (Bluetooth) IMU and the cellphone's built-in camera.

* *AndroidIns* - An Android app. that performs inertial navigation w/ zero velocity corrections via a Kalman filter using the built-in gyroscope and accelerometers in a cellphone.

5. SOFTWARE DEPENDENCIES
------------------------

This software has the following dependencies.  Depending on the exact configuration only a subset of these may be required in a given component:

* BASIL: Background software for inertial navigation and Kalman filtering available for license from Carnegie Mellon University.  Contact Michael George, mgeorge@cmu.edu and the CMU Center for Technology Transfer and Enterprise Creation (CTTEC) via innovation@cmu.edu.

* Eigen (Tested w/ version 3.2.0): [http://eigen.tuxfamily.org/index.php?title=Main_Page](http://eigen.tuxfamily.org/index.php?title=Main_Page)

* Boost (Tested w/ version 1.54):  [http://www.boost.org/](http://www.boost.org/)

* MOOS (Tested w/ svn revision 2300):  [http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php/Main/Download](http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php/Main/Download)

* FABMAP (Tested w/ version 2.0):  [http://www.robots.ox.ac.uk/~mjc/Software.htm](http://www.robots.ox.ac.uk/~mjc/Software.htm)

* OpenCV (Tested w/ version 2.4.6):  [http://opencv.org](http://opencv.org)

* Xsens MT SDK (For interfacing to Xsens IMUs):  [http://www.xsens.com](http://www.xsens.com)

* FlyCap2 (For interfacing to Pt. Grey cameras):  [http://www.ptgrey.com](http://www.ptgrey.com)

* Google Play Services (For access to the Google Maps Android API):  [http://developer.android.com/google/play-services/maps.html](http://developer.android.com/google/play-services/maps.html)

* Android Support Libraries (Tested w/ version 4): [http://developer.android.com/tools/support-library/index.html](http://developer.android.com/tools/support-library/index.html)

6. HARDWARE REQUIREMENTS
------------------------

* *ShoppingNavigator* requires an Xsens IMU ( [link](http://www.xsens.com)) and a Pt. Grey camera ([link](http://www.ptgrey.com)).  Software was tested with the Xsens MTi and a Pt. Grey FireFly IEEE 1397 camera.

* *wImuConsole* requires an InvenSense MPU9150 IMU ( [link](http://www.invensense.com/)) w/ a custom bluetooth interface and an Android cellphone.  Software was tested with a Google Nexus 4 phone running Android 4.2.2 API 17.  The custom IMU can be licensed by contacting mgeorge@cmu.edu and the CMU Center for Technology Transfer and Enterprise Creation (CTTEC) via innovation@cmu.edu.

* *AndroidIns* requires an Android cellphone.  Software was tested with a Google Nexus 4 phone running Android 4.2.2 API 17.  AndroidIns also requires the libAndroidIns.so binary which is included for devices with ARM based CPUs.   

7. BUILDING
-----------

* *ShoppingNavigator* can be built (assuming all depedencies are installed) via: 
    1. cmake
    2. make
* *wImuConsole* and *AndroidIns* will be built automatically when loaded in a android development environment such as the Eclipse ADT plugin: [http://developer.android.com/tools/sdk/eclipse-adt.html](http://developer.android.com/tools/sdk/eclipse-adt.html)

8. RUNNING
----------
* *ShoppingNavigator* contains a launch script in the binary sub-directory that will launch several components, a MOOS middleware instance to facilitate communication between components, an InertialTracker component which contains the inertial navigation and filtering components and an instance of the FABMAP system for place recognition based on the current camera view and a pre-built map of the environment of interest.
* *wImuConsole* and *AndroidIns* must be loaded onto an Android device (See [http://developer.android.com/tools/device.html](http://developer.android.com/tools/device.html) for details) and launched by clicking the relevant icon.
