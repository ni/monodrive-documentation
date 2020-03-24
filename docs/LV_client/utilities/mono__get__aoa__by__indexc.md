## mono_get_aoa_by_index.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__get__aoa__by__indexc.png" 
width="400"  />
</p>

### Description 
Obtain the closest Aoa in the radar output to a given Aoa from the ground truth output. Obain closes value and index. 

### Inputs
- **Radar_values(DBL 1D array):** The Aoa list for the targets detected.
- **GT_element(DBL):** The Aoa of the specific target tracked.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **closest_value(DBL):** Closest Aoa value from the radar output.
- **IndexClosestValue(I32):** Index of the closest Aoa from the radar output. .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
