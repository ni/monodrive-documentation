# Data Collection

## Quick Start Guide

This guide helps users become familiar with the monoDrive Real-to-Virtual data
collection process. The data collection needs to be conducted with the minimum:

* LiDAR - [Velodyne Puck Hi-Res](https://velodynelidar.com/products/puck-hi-res/) or better
* Camera for scene generation - [FLIR Blackfly](https://www.flir.com/products/blackfly-s-usb3/?model=BFS-U3-63S4C-C)
* Camera for dynamic actors - [FLIR Ladybug 5+](https://www.flir.com/products/ladybug5plus/)
* GNSS or GPS + IMU - [VectorNav VN-310](https://www.vectornav.com/products/vn-310) or better

The monoDrive Real-to-Virtual hardware provides the proper sensor suite and 
transforms in a package that can be easily mounted to a roof mount:

<div class="img_container">
    <img class='lg_img' src="../imgs/real_to_virtual_hardware_side_view.png"/>
</div>

## Sensor Installation and Calibration

The Real-to-Virtual system requires that sensors be placed so they can collect
high-resolution data of the road in front of the vehicle for scene generation. 
If dynamic actor detection is also required, a spherical camera sensor must be 
placed above the vehicle so all surroundings can be recorded. 

### Ladybug Calibration 

For the monoDrive Real-to-Virtual hardware, the original spherical camera data
must be calibrated for proper field-of-view prior to parsing in order to crop
out the data collection vehicle. The original full size images show a good 
portion of the data collection vehicle:

<div class="img_container">
    <img class='lg_img' src="../imgs/kitti_data_full_size_image.png"/>
</div>

The data collection software provides a tool to determine the optimal 
field-of-view in both the horizontal and vertical dimensions. 

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
    <img class='wide_img' src="../imgs/kitti_data_cropped_size_image.png"/>
</div>

### GNSS Calibration

In order to achieve an accurate heading from the VectorNav VN-310, the device
needs to run for a few minutes in order to achieve a sufficient navigation lock. 
For the monoDrive Real-to-Virtual hardware kit, this means plugging in the 
sensor kit and driving around for a few minutes prior to recording. See the 
[VectorNav Documentation](https://www.vectornav.com/resources/vn-310-user-manual) 
for more info on ensuring the device is properly locked.

## Data Collection

To collect data:

1. Ensure that the JSON configurations are correct in the "Configure" tab.
1. With the "Control" tab selected, click the "Run" arrow on the top-left of 
the monoDrive LabVIEW data logger application. The data logger will connect to 
all sensors and ensure they are functioning as expected.
1. The software is now connected and can display sensor outputs. 
To begin recording data, click "Record".
1. When data collection is complete, click "Stop Rec".
1. When you are done collecting data, click "Exit" to properly shut down all the 
sensor connections.

<div class="img_container">
    <img class='wide_img' src="../imgs/data_logger_main_tab.png"/>
</div>

## Driving Tips

The monoDrive Real-to-Virtual data processing works best for certain driving
conditions. In order to get the best results during data collection:

* Drive the vehicle between 15 and 50 miles per hour
* Avoid taking sharp or fast turns
* If possible avoid stopping the vehicle for long periods of time
* The less traffic around during collection, the better the output of the scene 
generation will be
* Try to limit data collection to a few minutes. While long runs will work, 
shorter and more frequent data collection will make processing easier

## Verifying Data Collection Output

Several data stream files will be saved by the monoDrive Real-to-Virtual data
logger. By default the logger will save data in the folder specified in the UI 
in subfolders that match the year, date, and time by hour. To verify that a run 
was successful:

* Ensure that all streams are present in the output `binary` directory:
    * Configuration files: `configuration_vectornav.txt`, 
    `configuration_ladybug.txt`, `configuration_lidar.txt`
    * LiDAR data: `lidar.bin`
    * GNSS data: `vectornav.bin`
    * Ladybug Data: `streams-000000.pgr`
    * Timestamp data: `timestamps_lidar.txt`, `timestamps_trigger.txt`

## Parsing Data Into KITTI Format

For use in the monoDrive Real-to-Virtual data processing pipeline, the collected
data needs to be converted to the 
[KITTI format](http://www.cvlibs.net/datasets/kitti/). The 
`monodrive_to_kitti.py` script in the monoDrive Mapping Toolkit allows users
to parse the collected binary data into KITTI:

```
$ conda activate mapping_toolkit
(mapping_toolkit)
$ python monodrive_to_kitti.py --input /path/to/data/binary --output /path/to/data/kitti --crop
```

When complete, the data will be moved to the `kitti` directory under the same 
data collection directory.

<p>&nbsp;</p>
