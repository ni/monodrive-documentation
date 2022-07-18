# mono_filter_average_by_target.vi

<p class="img_container">
<img class="lg_img" src="../mono_filter_average_by_target.png"/>
</p>

### Description

Filter targets and calculate average for AoA, range and velocity for the specified target id when there are multiple targets with the same target id.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Target id to track:**  Target id to filter from the target list
 

- **Targets:**  Target list, this can be the Ground Truth list or the Target
list
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Average AoA:**  Average Angle of Arrival for the specified target.
 

- **Average Range:**  Average range for the specified target.
 

- **Average Velocity:**  Average velocity for the specified target.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
