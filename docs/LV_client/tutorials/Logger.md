# Logger and Parser
## mono_logger.vi
User should connect the output of the sensor to the **Data** input of the mono_logger.vi as follows depending on the type of sensor:   

|  Sensor  |  Connection  |   
|---|---|   
|  Any type of Camera | Raw data |   
|  RPM |Raw data  |   
|  State Sensor | State sensor sample  |   
|  Ultrasonic | Raw data  |   
|  LiDAR | Raw data  |   
|  Semantic LiDAR | Raw data  |   
|  Collision | Raw data  |   
|  IMU | Raw data  |   
|  GPS | Raw data  |   
|  Radar | Target list  |  
|  Radar Cube | Radar Data Cube  |   

User will also need to connect the **Configuration** output of the sensor to the **Configuration** input of the `mono_logger.vi`

User will need to specify any custom name for the log file, taking into account that the name of the file by default will be the type of the sensor logged. For instance, if there are two `Camera` sensors, only one file `Camera.bin` will be created, to avoid this from happening, the user will need to specify a name for the second camera sensor, i.e. `Frontal` so that the second file is created with the name `Camera_Frontal.bin`.


This VI will produce a binary file for the following sensors:   

   - IMU   
   - LiDAR   
   - Any type of Camera   
   - GPS   
   - RPM   
   - Radar Cube   
  
This VI will produce a text file for: 

   - Ultrasonic   
   - State Sensor   
   - Collision   
   - Radar   
   - Camera annotation (if Annotation flag set to true)   

### Example
<p class="img_container">
<img class="lg_img" src="../logger.png"/>
</p>

## mono_parse_binaries.vi

To visualize data from a binary file:

1. Browse to the configuration file to be parsed.
2. The program will find the correspoding binary file.   

### Camera data
For camera images the user needs to specify a path to save the image. If the folder does not exist, the folder will be created.

Then the user will need to select the file format if he/she wants to save as a JPEG or PNG formatted image. By default the image file format is saved as PNG.

### LiDAR
For LiDAR, the user needs to specify the VeloView port he/she wants to stream for the LiDAR data
