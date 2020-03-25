## mono_kf_init.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/shared_libraries/mono__kf__initc.png" 
width="400"  />
</p>

### Description 
Initialize a `KalmanFilter` object.

### Inputs
- **R(DBL):** Measurement noise covariance.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **DLL path out(Path:** Path to the Release or Debug DLL.  .
- **inti:** return value if executed succesfully .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
