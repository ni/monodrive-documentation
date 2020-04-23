# Vehicles

## Vehicle
```
{
  "id": "laptop_test",
  "server_ip": "127.0.0.1",
  "server_port": 8998,
  "client_ip": "127.0.0.1",
  "lane_number": 0,
  "position": 22000,
  "spawning_rotation": {
        "pitch":0.0,
        "yaw":-90.0,
        "roll":0.0
  },
  "mesh_path": "/Game/Vehicles_Vol1/Models/SUV/SUV_rigged.SUV_rigged",
  "anim_path": "/Game/Vehicles_Vol1/Blueprints/SUV_AnimBP.SUV_AnimBP_C",
  "wheels":[
    {
      "id": "FL_Wheel",
      "wheel_number": 0,
      "offset": {
        "x":0.0,
        "y":-12.0,
        "z":0.0
      },
      "front": true
    },
    {
      "id": "FR_Wheel",
      "wheel_number": 1,
      "offset": {
        "x":0.0,
        "y":12.0,
        "z":0.0
      },
      "front": true
    },
    {
      "id": "BL_Wheel",
      "wheel_number": 2,
      "offset": {
        "x":0.0,
        "y":-12.0,
        "z":0.0
      },
      "front": false
    },
    {
      "id": "BR_Wheel",
      "wheel_number": 3,
      "offset": {
        "x":0.0,
        "y":12.0,
        "z":0.0
      },
      "front": false
    }
  ],
  "sensors": [
        {
      "type": "Lidar",
      "id": "far_left",
      "packet_size": 1472,
      "listen_port": 8088,
      "display_process": false,
      "sensor_process": true,
      "location": {
        "x":-75.0,
        "y":-25.0,
        "z":350.0
      },
      "rotation": {
        "pitch":0.0,
        "yaw":0.0,
        "roll":0.0
      },
      "max_distance": 8000.0,
      "vertical_fov_angle": 30.0,
      "horizontal_resolution": 0.4,
      "fps": 10.0,
      "n_lasers": 32,
      "reset_angle": 0.0
    }
  ]
}
```
<p>&nbsp;</p>