![monoDrive](https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/monoDriveLogo.png "monoDrive")

# monoDrive client installation

# Linux
- [Install monoDrive Simulator](http://www.monodrive.io)
- `pip install virtualenv`
- `cd client`
- `virtualenv venv`
- `venv\Scripts\activate`
- `pip install -e . --upgrade`
- `python -c "import monodrive; print('Success')"`
- `python examples\test.py`

# Windows
- [Install monoDrive Simulator](http://www.monodrive.io)
- [Install Windows Desktop](https://desktop.github.com/)
- git clone https://github.com/monoDriveIO/client.git
- Python 3.5 is required for monoDrive Windows client
    - [Download Python 3.5](https://www.python.org/downloads/release/python-353/)
    - DOS Prompt (for Pycharm see below)
        - Setup Python Path
            -Go to `Control Panel\System and Security\System`
            select
            
        - Advanced system settings from the left panel 
        - from Advanced tab click on Environment Variables
        - In the System variables section search for (create if doesn't exist)
        - PYTHONPATH  and set
            `C:\Python35\;C:\Python35\Scripts;`
            
        - You need to restart CMD.
        - Verify that worked 
            - `which python` 
                - result:[path to the python you installed above]
            - `pip install virtualenv`
            - `cd client`
            - `virtualenv venv`
            - `venv\Scripts\activate`
            - `pip install -e . --upgrade`
            - `python -c "import monodrive; print('Success')"`
            - `python examples\test.py`

    - Pycharm (optional)
        - [Download PyCharm Community Edition](https://www.jetbrains.com/pycharm/download/#section=windows) and Install. 
        - Clone [PythonClient](https://github.com/monoDriveIO/PythonClient.git).
        - Open the root directory of the repo in PyCharm.
        - Go to `File` then `Settings`.
        - Expand the `Project: PythonClient` and select `Project Interpreter`.
        - Click the gear icon in the upper right and select `Create VirtualEnv`.
        - Set the field
          - Name: `python35`
          - Location: `C:\envs\python35`
          - Base interpreter: `C:\Python35\Python.exe`
        
        - Run `examples/test.py`

- OpenCV
    - [Download OpenCV Python](http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv).
    - Select the `opencv_python‑3.4.1+contrib‑cp35‑cp35m‑win_amd64.whl` version. *Note: The 3.4.1 may be different when new versions are released, don’t focus on the version number.*
    - Open Command Prompt and run `C:\envs\python35\Scripts\activate.bat`
    - CD to your downloads folder.
    - Run `pip install “opencv_python‑3.4.1+contrib‑cp35‑cp35m‑win_amd64.whl”` or whatever version of OpenCV you downloaded above.

# Simulator Configuration

- Open `examples/test.py`
- In the main method of the file there is an initialization of `simulator_config`. You can change which json file is used here.
- Open the configuration you want to use in the configurations folder and you can change the `server_ip` (the ip address of the computer running the simulator), the `server_port` (the port for the simulator, should be 8998), and the `client_ip` (the ip address of the computer running the simulator). If both simulator and client are running on the same computer use localhost as the ip addresses (127.0.0.1).
- In this configuration you can also change the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).


# Vehicle Configuration

- Open `examples/test.py`
- In the main method of the file there is an initialization of `vehicle_config`. You can change which json file is used here.
- The different configurations reside in the configurations folder.
- To add sensors open a configuration and go to the sensors array. You can copy an existing sensor if it is in the configuration, or you can open `cruise_demo_all.json` to find a sensor you are wanting to add, then modify the parameters to fit your needs.
- To remove a sensor open a configuration and go to the sensors array. Then just delete that dictionary for the sensor you want to remove, ensuring that you didn’t delete a comma between the dictionaries.


### Run
- Run `examples/test.py`
