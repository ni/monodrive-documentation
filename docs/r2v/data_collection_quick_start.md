# Data Collection Quick Start

## monoDrive Real-to-Virtual Data Collection Quick Start Guide

This guide helps users become familiar with the monoDrive Real-to-Virtual data
collection process. The data collection needs to be conducted with the minimum:

* LiDAR - [Velodyne Puck Hi-Res](https://velodynelidar.com/products/puck-hi-res/) or better
* Camera for scene generation - [FLIR Blackfly](https://www.flir.com/products/blackfly-s-usb3/?model=BFS-U3-63S4C-C)
* Camera for dynamic actors - [FLIR Ladybug 5+](https://www.flir.com/products/ladybug5plus/)
* GNSS or GPS + IMU - [Xsens MTi-G-710](https://shop-us.xsens.com/shop/mti-g-710/mti-g-710-gnssins-2a8g4) or better

The monoDrive Real-to-Virtual hardware provides the proper sensor suite and 
transforms in a package that can be easily mounted to a roof mount:

<div class="img_container">
    <img class='lg_img' src="../imgs/real_to_virtual_hardware_side_view.png"/>
</div>

### Sensor Installation and Calibration

The Real-to-Virtual system requires that sensors be placed so they can collect
high-resolution data of the road in front of the vehicle for scene generation. 
If dynamic actor detection is also required, a spherical camera sensor must be 
placed above the vehicle so all surroundings can be recorded. 

#### Ladybug Calibration 

For the monoDrive Real-to-Virtual hardware, the original spherical camera data
must be calibrated for proper field-of-view prior to parsing in order to crop
out static objects. The orignal shows a good portion of the data collection 
vehicle:

<div class="img_container">
    <img class='lg_img' src="../imgs/kitti_data_full_size_image.png"/>
</div>

The data collection software provides a tool to determine the optimal 
field-of-view in both the horizontal and vertial dimensions. 

> **TODO:** insert picture of calibration tool for images.

After finding the optimal field-of-view angles, the values should be entered 
into the "Ladybug Configuration" section of the "Configuration" tab. For the
monoDrive vehicle the JSON should look like:

```
[
    {
    "type": "Ladybug",
    "location": {
      "x": 0,
      "y": 0,
      "z": 0
    },
    "rotation": {
      "pitch":0.0,
      "yaw":0.0,
      "roll":0.0
    },
    
    "desired_hfov": 90.0,
    "desired_vfov": 62.0
    }
]
```

When parsed, the above settings turn the image into a cropped version:

<div class="img_container">
    <img class='wide_img' src="../imgs/kitti_data_cropped_image.png"/>
</div>

#### GNSS Calibration

In order to achieve an accurate heading from the Xsens GNSS, the magnetic field
mapper tool must be  used to calibrate for the local magnetic bias. The 
[Magnetic Calibration Manual](https://www.xsens.com/hubfs/Downloads/Manuals/MT_Magnetic_Calibration_Manual.pdf) Section 3.3 specifies the steps necessary
to calibrate. 

1. Open that Xsens Magnetic Field Mapper software
1. Select "Use Motion Tracker"
1. "Scan" for the connected Xsens device
1. Once the device is found, click next "Start" to begin calibration
1. Drive the vehicle in **two consecutive circles** then click "Process"
1. If result state "Acceptable" or better, select "Write to selected devices"

### Data Collection

After calibration, **do not power off the hardware**. This will preserve all
calibration parameters. To collect data:

1. Click the "Run" arrow on the top-left of the monoDrive LabVIEW data logger application. The data logger will connect to all sensors and ensure they are functioning as expected.
1. Click "Configure" to configure the software the values in the configuration tab.
1. The software is now connected and can display sensor outputs. In order to acclimate the GNSS to the magnetic field, drive around for approximately 5 minutes
1. Once the IMU yaw and the Estimated Yaw are relatively close (~1-2 degrees), data collection can begin
1. To begin recording data, click "Record"
1. When data collection is complete, click "Stop"

<div class="img_container">
    <img class='wide_img' src="../imgs/data_logger_main_tab.png"/>
</div>

### Driving Tips

The monoDrive Real-to-Virtual data processing works best for certain driving
conditions. In order to get the best results during data collection:

* Drive the vehicle between 15 and 50 miles per hour
* Avoid taking sharp or fast turns
* If possible avoid stopping the vehicle for long periods of time
* The less traffic around during collection, the better the output of the scene generation will be
* Try to limit data collection to a few minutes. While long runs will work, shorter and more frequent data collection will make processing easier

### Verifying Data Collection Output

Several data stream files will be saved by the monoDrive Real-to-Virtual data
logger. By default the logger will save data in the folder specified in the UI 
in subfolders that match the year, date, and time by hour. To verify that a run 
was successful:

* Ensure that all streams are present in the output `binary` directory:
    * Configuration files: `configuration_gnss.txt`, `configuration_ladybug.txt`, `configuration_lidar.txt`
    * LiDAR data: `lidar.bin`
    * GNSS data: `logfile.mtb`
    * Ladybug Data: `streams-000000.pgr`
    * Timestamp data: `timestamps_lidar.txt`, `timestamps_trigger.txt`

Validating the location of the data collection can be done by viewing the Xsens
output in [Google Earth](https://www.google.com/earth/versions/#download-pro):

1. Double-click the `logfile.mtb` file for the data collection
1. In the MT Manger, select `File` -> `Export`
1. Select "KMZ Exporter" from the "Exporter" drop-down and click "Export"
1. Find the `logfile-000.kmz` that was generated and double click. Google Earth will open showing the data collection.
1. Ensure the location of the data collection is correct and the heading arrow points along the path similar to the image below

<div class="img_container">
    <img class='lg_img' src="../imgs/google_earth_gnss_output.png"/>
</div>

### Parsing Data Into KITTI Format

For use in the monoDrive Real-to-Virtual data processing pipeline, the collected
data needs to be converted to the 
[KITTI format](http://www.cvlibs.net/datasets/kitti/index.php). The `monodrive_to_kitti.py` script in the monoDrive Mapping Toolkit allows users
to parse the collected binary data into KITTI:

```
$ conda activate mapping_toolkit
(mapping_toolkit)
$ python monodrive_to_kitti.py --input /path/to/data/binary --output /path/to/data/kitti --crop
```

When complete, the data will be moved to the `kitti` directory under the same 
data collection directory.
