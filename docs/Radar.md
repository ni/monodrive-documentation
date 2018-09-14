## Radar Sensor

The configuration for a radar sensor.

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/radarsensor.PNG" />
</p>

```
{
      "type": string,
      "id": string,
      "packet_size": int,
      "listen_port": int,
      "display_process": bool,
      "sensor_process": bool,
      "location": {
        "x": float,
        "y": float,
        "z": float
      },
      "rotation": {
        "pitch": float,
        "yaw": float,
        "roll": float
      },
      "num_samples_per_sweep": int,
      "fs": int,
      "fc": float,
      "num_sweeps": int,
      "range_max": float,
      "sweep_num_for_range_max": float,
      "range_resolution": float,
      "max_velocity": float,
      "fps": int,
      "transmitter": {
        "aperture": float,
        "gain": float
      },
      "receiver": {
        "aperture": float,
        "nf": float,
        "noise_temp": float,
        "nb": float,
        "gain": float,
        "kb": float
      }
}
```

- **num_samples_per_sweep**: 
- **fs**: 
- **fc**:  
- **num_sweeps**: 
- **range_max**: 
- **sweep_num_for_range_max**: 
- **range_resolution**: 
- **max_velocity**: 
- **transmitter**:
  - *aperture*: 
  - *gain*: 
- **receiver**:
  - *aperture*: 
  - *nf*: 
  - *noise_temp*: 
  - *nb*: 
  - *gain*: 
  - *kb*: 

### Output Data

```
# See [base sensor](Base-Sensor.md) for examples on how to get the sensor.
data_radar = radar.get_message()

# The data in data_radar is a dictionary.
game_time = data_radar['game_time']
data = data_radar['data']
```

### Raw Output Data Format

- **0...NumberOfSweeps as i:**
  - **Bytes 8i-4+8i:** Real bits represented as a float in single precision IEEE-754 format.
  - **Bytes 4+8i-8+8i:** Imaginary bits represented as a float in single precision IEEE-754 format.