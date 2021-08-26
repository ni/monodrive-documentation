# C++ Quick Start

The monoDrive C++ Client is Open Source Software for connecting to and 
configuring the monoDrive Simulator and Scenario Editor. To get started, 
contact support@monodrive.io to get access to the monoDrive Client repository,
then clone the repository. 

## Setup

The monoDrive C++ Client is cross-platform and tested on both Windows 10 and 
Ubuntu 18.04.

## Windows

### Windows Prerequisites

- Windows 10
- [Visual Studio 2019 Community Edition](https://visualstudio.microsoft.com/vs/community/)
- [VSCode](https://code.visualstudio.com/)

### Windows Library Dependencies 
**NOTE**: Extract or install these libraries to `C:\local` so cmake can find them.

- [Boost](https://sourceforge.net/projects/boost/files/boost-binaries/1.73.0/boost_1_73_0-msvc-14.2-64.exe/download)
  - Add `C:\local\boost_1_73_0\lib64-msvc-14.2` to your PATH variable
  - Create the `BOOST_ROOT` environment variable and set it to `C:\local\boost_1_73_0\`

<div class="img_container">
    <img class="lg_img" src="../imgs/boost_root_system_var.jpeg">
</div>

- [CMake](https://cmake.org/download/)
    - When installing, make sure to choose the option to CMake to your Windows Path variable.
 
To build the examples the following are required:

- [OpenCV](https://github.com/opencv/opencv/releases/download/4.3.0/opencv-4.3.0-vc14_vc15.exe) Extract to `C:\local\opencv` and add `C:\local\opencv\build\x64\vc15\bin` to your PATH environment variable.

- [Eigen](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip) Extract to `C:\local\Eigen3` and add `C:\local\Eigen3` to your PATH environment variable.
     - **NOTE**: You will need to move the extracted contents out of the version subfolder (eg `.\eigen-3.3.7\`) into the parent directory, such that `C:\local\Eigen3\Eigen` is a valid directory path.

<div class="img_container">
    <img class="lg_img" src="../imgs/eigen3_dir.jpeg">
</div>

### Environment Variables

<div class="img_container">
    <img class="lg_img" src="../imgs/env_paths.jpeg">
</div>

## Ubuntu

### Ubuntu 18.04 Prerequisites
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04.4/)
    - Run the prerequisite setup script from the root repository directory. This will install all dependencies and also build a newer version of Boost:
```bash
$ ./util/setup.sh
```

- [VSCode](https://code.visualstudio.com/)

### Ubuntu 20.04 Prerequisites
- Ubuntu 20.04
    - Install packages:
```bash
$ sudo apt-get update && sudo apt-get install libboost-dev libboost-system-dev libboost-filesystem-dev build-essential libeigen3-dev
```
- [VSCode](https://code.visualstudio.com/)

### Installation with Bazel
You can include the monoDrive Simulator client in your existing Bazel project
by adding the following lines to your `WORKSPACE` file
```
local_repository(
    name = "monodrive",
    path = "path/to/monodrive-client"
)
```
and the following to your `BUILD` files as needed
```
cc_library(
    name = "my_lib",
    srcs = [...],
    hdrs = [...],
    deps = [
        ...,
        "@monodrive//monodrive/core:monodrive"
    ]
)
```

### Installation to system for Bazel & Ubuntu
You can build and install the monoDrive Simulator client to your Ubuntu system
using CMake
```
mkdir build
cd build
cmake ..
make
sudo make install
```

This will install the client library under the prefix `/usr/local/monodrive/client`.
It can now be included or linked as needed.

For example, to compile your own executable with the monoDrive client library
```
g++ main.cpp -I/usr/local/monodrive/client/include/ -L/usr/local/monodrive/client/lib -lboost_system -lmonodrive -o my_program
```

Make library available for dynamic loading
```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/monodrive/client/lib
```

Finally run
```
./my_program
```


## Setup from Command Line

### Configure

To configure the CMake project, navigate to the project root, and run:
```
mkdir build
cd build
cmake ..
```

### Build

To build the core libraries and examples, from `./build`, run:

_Windows_
```
cmake --build .
```

_Linux_
```
make
```

### Run

To run an example, navigate back to the project root, and run:

_Windows_
```
./build/examples/cpp/lane_follower/Release/real_time.exe
```

_Linux_
```
./build/examples/cpp/lane_follower/real_time
```


## Setup using VSCode

### Build
1. Install and launch [VSCode](https://code.visualstudio.com/).

2. Add the following VSCode extensions:
    - [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake)
    - [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
    - [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)


3. Select `File -> Open Folder` and navigate to this folder to build the cpp-examples or simulator-cpp-client to build just the client library.
4. Use the CMake extension to configure and build
    1. Click the Configure All Projects icon:

    <div class="img_container">
        <img class="lg_img" src="../imgs/configure.png">
    </div>

    2. If prompted to Scan for Kits select Yes.

        **Windows**: Use `Visual Studio Community 2019 Release - amd64`.

        **Linux**: Use the compiler of your choice. Tested with `g++ 7.5.0`.

    3. Build the client by clicking the `Build All Projects` icon:

    <div class="img_container">
        <img class="lg_img" src="../imgs/build.1.png">
    </div>

### Set Launch Target

<div class="img_container">
    <img class="lg_img" src="../imgs/dev_target_set_example_v3.jpeg">
</div>

**NOTE**: Launching automatically fires off a build check in cmake so just setting the launch target will suffice.


### Run Windows Example

<div class="img_container">
    <img class="wide_img" src="../imgs/example_windows_vscode.jpeg">
</div>

*Example configuration with `fisheye_camera_equidistant` as a build + launch target.*

After you've set your build configuration and target, you can run by hitting `F5` to run in debug or `Ctrl+F5` to run without debugging.

## Installation with CMake
You can include the monoDrive Simulator client in your existing CMake project
by adding the following lines to your `CMakeLists.txt`

```cmake
# add monodrive client library from local repo
add_subdirectory(path/to/monodrive-client/monodrive mdclient)

# link targets as needed
target_link_libraries(<mytarget> monodrive)
```
