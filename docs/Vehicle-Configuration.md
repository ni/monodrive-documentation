### Example JSON
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

- Open `examples/test.py`
- In the main method of the file there is an initialization of `vehicle_config`. You can change which json file is used here.
- The different configurations reside in the configurations folder.
- To add sensors open a configuration and go to the sensors array. You can copy an existing sensor if it is in the configuration, or you can open `cruise_demo_all.json` to find a sensor you are wanting to add, then modify the parameters to fit your needs.
- To remove a sensor open a configuration and go to the sensors array. Then just delete that dictionary for the sensor you want to remove, ensuring that you didn’t delete a comma between the dictionaries.