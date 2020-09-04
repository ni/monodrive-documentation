# mono_logger.vi

<p class="img_container">
<img class="lg_img" src="../mono_logger.png"/>
</p>

### Description

This VI creates text files or binary files depending on the type of sensor.    

- **Text files**:   
   - Ultrasonic   
   - State Sensor   
   - Collision   
   - Radar   
   - Camera annotation (if Annotation flag set to true)   
   
- **Binary files**:   
   - IMU   
   - LiDAR   
   - Camera   
   - GPS   
   - RPM   
   - Radar Cube   
  

It also saves a text file for the camera annotation if the user set to True the annotation flag.  

### Inputs

- **Path to log:**  Directory path to save the binary files and text files
created
 

- **Data:**  Data to log into the binary or text file
 

- **Sensor Type:**  Type of sensor to log data from.
 

- **Name:**  The name to assign to the binary or text file. If empty, it
will assign the same name as the type of sensor.
 

- **Annotation:**  True to create a text file to save the annotation data of
the a Camera sensor.
 

- **Annotation content:**  Annotation text to save into the annotation text file. 
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
