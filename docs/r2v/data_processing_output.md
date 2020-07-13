# Data Processing Interim Data Products

## The Real-to-Virtual Pipeline and Data Products

The monoDrive Real-to-Virtual system operates as a pipeline of computer vision 
and machine learning algorithms in conjunction with several deep learning 
networks in order to produce the final Unreal Engine assets. Each step in the
pipeline outputs a data product that is then used by the next stage of the 
system. Each of these interim data products play an important role in the 
process of producing highly accurate simulation assets, but individually they
can also be useful to the end-user for tasks like ground-truthing, labeling, 
and training.

This page goes through the currently available data products from the monoDrive
Real-to-Virtual pipeline that is deployed to customers. 

## monoDrive Binary Data

The binary data that is recorded by the monoDrive Real-to-Virtual Data Logger 
consists of raw binary streams of data from the GNSS, Camera, and LiDAR systems
that are part of the monoDrive Real-to-Virtual hardware. The current data
logger saves:

* GNSS Data - Log files that can be played back in the 
[MT Software Suite](https://content.xsens.com/mt-software-suite-download?hsCtaTracking=ca04936d-827e-43ef-a5e9-1797c4d9a297%7C1e7a3b81-a8f1-40af-8464-97db700b3dec)
* Ladybug 5+ Camera Streams - These are `.pgr` stream files that contain all
6 cameras on the Ladybug Camera that can be played back with the 
[Ladybug SDK](https://www.flir.com/products/ladybug-sdk/)
* Binary LiDAR Data - This is a raw stream of the packets sent from the 
Velodyne LiDAR. The monoDrive Real-to-Virtual Data Logger provides software to 
play this data back so that it can be viewed in 
[Veloview](https://www.paraview.org/veloview/).
* Configuration Files - These are text files produced by the Data Logger 
software that contain orientation information for each sensor during parsing
* Timestamp Files - These contain the timestamps for each data acquisition 
iteration.

## KITTI Formatted Data

After parsing the monoDrive Real-to-Virtua binary data with the monoDrive 
[mapping-toolkit](https://github.com/monoDriveIO/mapping-toolkit), the data
is formatted into the 
[KITTI format](http://www.cvlibs.net/datasets/kitti/raw_data.php). This format
is used by several different benchmarks and algorithms for testing various 
vehicle perception stacks.

## Point Cloud Data

The point cloud directory contains:

* KML Files and txt files for path
* Invidiually semantically colored point clouds
* Stitched clouds for each semantic label
* Fully stitched clouds

## Segmentation

## Redaction

## ORB SLAM

## Object Detection

## Tracking

# Mesh
