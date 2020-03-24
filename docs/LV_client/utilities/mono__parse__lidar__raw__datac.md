## parse_lidar_raw_data.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__parse_lidar_raw_data.png" width="400"  />
</p>

### Description 
Obtains timestamp (position 1200) in microseconds from the raw lidar stream (1206 bytes).

### Inputs
- **Data in:** Lidar raw data packet with 1206 bytes.
- **offset(1200):** Index in the lidar stream where the timestamp is located.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **timestamp (us):** The amount of microseconds past the hour.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
