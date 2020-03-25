## mono_ncap_sensor.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/mono__ncap__sensorc.png" 
width="400"  />
</p>

### Description 
Configure and sample the NCAP sensor. The NCAP sensor records relevant information from the cars in the scene with the tag `vt`.
It reports if a collision was present during the simulation.

### Inputs
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **data (1D String Array):** relevant information from the cars in the scene with the tag `vt`. 
It reports if a collision was present during the simulation.
- **NCAP Configuration:**  .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
