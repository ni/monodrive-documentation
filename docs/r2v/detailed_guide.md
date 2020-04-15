# Detailed Guide

## monoDrive Real-to-Virtual Detailed Guide

### Data Collection
The monoDrive Real-to-Virtual (R2V) tool consists of a suite of hardware and 
software for collecting real-world data in order to produce artifacts usable in 
the virtual simulation environments such as monoDrive Simulator and the Unreal 
Engine. The hardware (R2V Sensor Kit), is mounted on a data collection vehicle. 
It consists of an array of calibrated cameras, an inertial measurement unit with 
GNSS capability, and a 360 degree LiDAR. The data collection may be performed 
on a laptop with USB 3.0 and gigabit ethernet connected to the sensors and 
running the monoDrive Real-to-Virtual data collection software.

### KITTI Storage Format 
The R2V data collection process stores raw image, GNSS, and LiDAR point cloud 
data in binary format. This raw capture sensor data is then converted to KITTI 
format (see http://www.cvlibs.net/datasets/kitti/index.php for more information 
about the KITTI project and format). Any captured sensor data in the KITTI 
format is the handoff point to the monoDrive R2V data processing pipeline in 
order to produce usable simulation assets. KITTI formatted data is consumable 
by the monoDrive R2V pipeline and provides the necessary information for 
producing virtual environments that are indexed in the monoDrive GeoDatabase. 

### R2V Pipeline
After data collection, the data is uploaded to and processed by monoDrive’s
proprietary Real-to-Virtual data processing pipeline to produce assets for use 
in virtual environments. The R2V Pipeline produces road surfaces, static 
object locations, detected lane locations, and dynamic actor trajectory files 
(a format usable for replay in the monoDrive Simulator).  These assets are 
indexed with the monoDrive GeoDatabase where they are integrated with elevation 
and satellite imagery. Upon completion of the R2V pipeline processing, the 
end-user is able to query the GeoDatabase to retrieve all data products in
geodetically registered 500 square meter tiles in Universal Transverse Mercator 
(UTM) coordinates. The UTM Zone is determined from the GNSS information from 
the KITTI dataset to be processed.

This document focuses on utilizing the monoDrive R2V sensor suite in order to 
collect high quality, time synchronized data. 

### Sensor Configurations Requirements
The current release of the monoDrive R2V system includes a suite of hardware 
(shown in Figure 1) consisting of:

* FLIR Ladybug 5+
* Velodyne Puck Hi-Res with GPS antennae 
* Xsens MTi-G-710 with GPS antennae 
* Adjustable roof mount
* monoDrive Custom Wiring Configuration (for sensor synchronization)
* Laptop computer with SSD storage drive, USB 3.0, and gigabit Ethernet

<div class="img_container">
    <img class='lg_img' src="../imgs/real_to_virtual_hardware_side_view.png"/>
</div>

### Sensor Orientation and Calibration

The monoDrive sensor suite is typically mounted on a vehicle rack on top of a 
data collection vehicle near the front cabin. This position provides enough 
visibility for the camera and LiDAR to image the road immediately in front of 
the vehicle and acquire the data necessary to create realistic assets.

When mounted, the Ladybug “Camera 0” (indicated by the “0” engraved on the 
housing) should be pointed towards the front of the vehicle. This will orient 
all of the sensors so that they can properly measure the inertial movement of 
the vehicle as it travels. Figure 2 shows the R2V sensor suite from a 
right-side view. 

Note that the LiDAR is adjustable from a 10 to a 20 degree tilt. It is 
important to record the setting of LiDAR position prior to data collection so 
that the information can be configured in the data logging tool (see the 
“Data Logger” section for more information on configuration parameters). To 
adjust the LiDAR angle, use an allen wrench to loosen both the left and right 
screws holding the LiDAR in position. The LiDar is tilted up and down to one of 
the 6 preset markers, shown in Figure 2, indicating the angle of tilt. While 
the LiDAR is in the desired position (normally full tilt 20 degrees), tighten 
the allen screws back down to secure the LiDAR.

<div class="img_container">
    <img class='wide_img' src="../imgs/real_to_virtual_hardware_close_up.png"/>
</div>

Prior to data collection, it is important to calibrate the GNSS located on or 
inside the R2V data collection system’s column. Xsens provides a “Magnetic 
Field Mapper” utility that comes pre-installed with the [Xsens SDK]
(https://content.xsens.com/mt-software-suite-download). This utility allows
calibration of the local magnetic field to achieve accurate heading estimation. 
Please refer to the Xsens “Magnetic Field Manual” (Document MT0202P, Revision 
M, 21 Nov 2018) section 3.3 for instructions on how to use the field mapping 
software. NOTE: When calibrating the GNSS magnetic field on top of a vehicle, 
you should drive in 2 consecutive circles in either direction.

### Data Logging Solution

After the sensor suite is connected and calibrated, the monoDrive R2V Data 
Logger is used to collect binary stream data on a laptop computer. The data 
logger, shown in Figure 3, is a LabView application which enables configuration 
of sensor settings and view mission critical sensor data during a logging 
session. After a logging session is complete, the binary sensor stream data can 
be found in the directory selected in the data logging application. This binary 
data includes all sensor stream data for Camera, LiDar, and GNSS.

<div class="img_container">
    <img class='lg_img' src="../imgs/data_logger_main_tab.png"/>
</div>

<div class="img_container">
    <img class='lg_img' src="../imgs/data_logger_config_tab.png"/>
</div>


### Recording Data
Prior to recording data, make sure the GNSS has been calibrated. For best 
results, run the recording software for a few minutes while driving. It is best 
to rotate the vehicle at least 360 degrees by turning around several corners to 
allow the GNSS to get acclimatized to the environment. 

#### Hardware Prerequisites:
* Connect the Ladybug Camera’s USB cable to a USB 3.0 port on the recording laptop
* Connect the GNSS USB cable from the R2V system’s data collection electrical box to a USB port on the recording computer.
* Connect an Ethernet cable to the R2V ethernet port on the data collection computer.
* On the data collection computer, make sure the Ethernet port connected to the R2V system is on a local IP of **192.168.1.200**

#### Data Logger Recording:
1. Open the r2v.lvprj file on the R2V folder
1. Double-click on the main.vi
1. Click on the arrow to run the main.vi
1. Select the path where the logs are going to be stored using the “Directory 
to Log” control: 
    1. The program will create the following directory structure to store your data:
`<Chosen Directory>/YYMMDD/HH_MM_SS/binary/`
1. Click on the configuration window and make sure the values are still valid for your setup (i.e. the tilt for lidar, the location of the GNSS, etc) 
1. When ready to boot the sensor suite, hit the button Configure. It is a good idea to ensure everything is working during this step by observing the data values on all the indicators.
1. When ready to record R2V data, click the button Record.
**NOTE:** By default there is a 5 second delay after you hit record to allow the sensors to initialize prior to recording.
1. When ready to stop recording data, click the button Stop Rec.
1. The system will wait for you to hit the button Configure and Record. 
1. When you are ready to finish collecting data, click the EXIT button, this will disconnect the GNSS and close the open connections.
**NOTE:** If the the LabView “Stop” button is pressed instead of the R2V EXIT button, the entire application will need to be restarted in order to record again.

### KITTI Formatted Data
The KITTI format is leveraged by the monoDrive R2V software to provide an industry accepted data format. This format is used widely in industry and academia to process image, LiDAR, and GNSS data for various Simultaneous Localization and Mapping (SLAM) algorithms and other computer vision work. For more information on the format itself, see the author’s website at http://www.cvlibs.net/datasets/kitti/index.php.

Since the recorded data must be streamed to file as raw binary in order to meet the frame rate requirements for R2V processing, the KITTI format is a data product that must be produced after data collection. The monoDrive KITTI parsing and formatting process reads all the data streams, correlates all data in time, and produces the transformation (extrinsic and intrinsic) data for the data collection run. 

#### Raw binary sensor data to KITTI
monoDrive provides a Python/C++ application that enables users to parse the binary stream data from the R2V Data Logger into the KITTI format. Access to the application’s code is available upon request. The README for the application describes installation and usage of the software.
