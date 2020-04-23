# ROS Quick Start

## monoDrive ROS Client

## Ubuntu 18.04 Prerequisites

- [monoDrive c++ client](https://github.com/monoDriveIO/monodrive-client/blob/master/cpp-client/README.md#monodrive-c++-client)

- [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) *Note: Tested with melodic*

- ROS Bridge: 

    ```bash
    $ sudo apt-get install ros-melodic-rosbridge-suite
    ```

### monoDrive ROS Packages and example build

1. Execute the following to build the ROS packages: 

    ```bash
    $ cd ros-examples
    $ catkin_make
    $ source devel/setup.bash
    ```

2. Add the setup file to your .bashrc to add the packages to your ros path on terminal load:

    ```bash
    $ echo "source <path/to/devel/setup.bash>" >> ~/.bashrc
    ```

### monoDrive Simulator and Client network setup

*If you are running both the client and simulator on the same machine you can skip this section as the networking defaults are for local host.*

If you are running the simulator and client on separate machines the following networking settings must be configured.

1. Set the IP address and port IDs for the machine running the simulator and the machine running the ros bridge

    - In the configuration file, `simulator_control/confg/simulator.json`, set the IP and port (default is `9090`) of the machine that will host the **ros bridge**:

        ```json
        "ros": {
            "port": 9090,
            "server": "192.168.86.167"
        },
        ```

    - and the IP and port of the **simulator**:

        ```json
        "server_ip": "192.168.86.168",
        "server_port": 8999,
        ```

2. Forward the ports on both machines (`9090` and `8999`) from step 1 or disable the firewalls on both machines.
