### Install Python
- [Download Python 3.5](https://www.python.org/downloads/release/python-353/), choose the Windows x86-64 executable installer.
- Launch installer.
- Select `Custom installation`.
- Select `pip`, `tcl/tk and IDLE`, and `Python test suite`.
- Change the install directory to `C:\Python35`

### Install PyCharm(or other IDE)
- [Download PyCharm Community Edition](https://www.jetbrains.com/pycharm/download/#section=windows) and Install. 
- Clone [Client](https://github.com/monoDriveIO/Client.git).
- Open the root directory of the repo in PyCharm.
- Go to `File` then `Settings`.
- Expand the `Project: PythonClient` and select `Project Interpreter`.
- Click the gear icon in the upper right and select `Create VirtualEnv`.
- Set the field
  - Name: `python35`
  - Location: `C:\envs\python35`
  - Base interpreter: `C:\Python35\python.exe`

If you're in an environment behind a firewall or don't have internet access during the install skip this next step.

- Open the `requirements.txt` file and wait for Package requires to populate in PyCharm and click `Install requirements`, but **uncheck opencv-python** before clicking go.

### On-Premise / Offline Steps
If you have normal internet access and PyCharm installed the requirements automatically for you skip these steps and move on to [OpenCV](#opencv)
- Go to `File` then `Settings`.
- Expand the `Build, Execution, Deployment > Console` and select `Python Console`.
- Verify that `Python Interpreter` is set to the same as what you setup in the previous step, typically `Python 3.5 (Client)`
- Hit `OK` to close Settings
- Go to `View > Tool Windows` then `Terminal`
- At the terminal prompt run `pip.exe install -r req-offline.txt --no-index --find-links=./libs/`

### OpenCV
- [Download OpenCV Python](http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv).
- Select the `opencv_python‑3.4.1+contrib‑cp35‑cp35m‑win_amd64.whl` version. *Note: The 3.4.1 may be different when new versions are released, don’t focus on the version number.*
- Open Command Prompt and run `C:\envs\python35\Scripts\activate.bat`
- CD to your downloads folder.
- Run `pip install “opencv_python‑3.4.1+contrib‑cp35‑cp35m‑win_amd64.whl”` or whatever version of OpenCV you downloaded above.

### Run
- Run `examples/test.py`
