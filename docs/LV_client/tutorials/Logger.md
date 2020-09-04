# Mono_logger.vi
User should connect the output of the sensor to the **Data** input of the mono_logger.vi as follows depending on the type of sensor:   

|  Sensor  |  Connection  |   
|---|---|   
|  Camera | Raw data |   
|  RPM |Raw data  |   
|  State Sensor | State sensor sample  |   
|  Ultrasonic | Raw data  |   
|  LiDAR | Raw data  |   
|  Semantic LiDAR | Raw data  |   
|  Collision | Raw data  |   
|  IMU | Raw data  |   
|  Radar | Target list  |  
|  Radar Cube | Radar Data Cube  |   


This VI will produce a binary file for the following sensors:   

   - IMU   
   - LiDAR   
   - Camera   
   - GPS   
   - RPM   
   - Radar Cube   
  
This VI will produce a text file for: 

   - Ultrasonic   
   - State Sensor   
   - Collision   
   - Radar   
   - Camera annotation (if Annotation flag set to true)   

## Example
<p class="img_container">
<img class="lg_img" src="../logger.png"/>
</p>

# Mono_parse_binaries.vi

This tool can help the user to visualize data from a binary file.

1. Browse to the file you want to parse.
2. Select the type of sensor you want to parse.
3. For camera images add a path to save the images. 
4. For LiDAR set the VeloView port you want to stream the LiDAR data.
