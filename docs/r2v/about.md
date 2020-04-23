# About Real-to-Virtual

## What is monoDrive Real-to-Virtual

The monoDrive Real-to-Virtual hardware and software provide an end-to-end 
solution for collecting real-world data from cameras, LiDAR, and GNSS systems
to create high-fidelity assets and maps usable in the Unreal Engine and the 
monoDrive Scenario Editor.

<video width=650px autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/r2v_comparison_converted.mp4" type="video/webm">
</video>

* Sensor fusion and machine learning algorithms construct 3D meshes from LiDAR and image data
* monoDrive Direct Texturing enables high-fidelity texturing of meshes for realistic roads
* monoDrive developed signal processing and machine learning algorithms enable static road art and dynamic vehicle placement in simulation

<video width=650px autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/dynamic_actor_tracking_converted.mp4" type="video/webm">
</video>

## Data Products

The monoDrive data collection streams binary data to disk at 10 to 20 Hz with
the provided LabVIEW data collection application. The data is then converted
into the [KITTI format](http://www.cvlibs.net/datasets/kitti/raw_data.php) for 
processing.

* End users have full access to KITTI data
* Raw assets from the pipeline provide road and static actor locations
* monoDrive Simulator Trajectory Files provide dynamic actor locations and replay ability

<div class="img_container">
    <img class='wide_img' src="../imgs/r2v_data_products.png"/>
</div>

### Unreal Engine Assets

The Real-to-Virtual system provides high-resolution textured roads for use in 
the Unreal Engine. The monoDrive GeoDatabase provides the satellite imagery
and elevation information for generating landscape meshes for anywhere in
the world. 

<div class="img_container">
    <img class='wide_img' src="../imgs/r2v_ue4_images.png"/>
</div>

Combining the Real-to-Virtual road meshes, monoDrive GeoDatabase satellite 
tiles, and static road art placement, the Real-to-Virtual system produces
Unreal Engine maps that are usable in the monoDrive Simulator and Scenario Editor.
The monoDrive clients can connect to the running simulator to replay the 
original dynamic actors from the generated Trajectory File or create their own,
new scenarios using the monoDrive Scenario Editor.

<video width=650px autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/r2v_ue4_sim_converted.mp4" type="video/webm">
</video>