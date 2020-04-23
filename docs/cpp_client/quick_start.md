# C++ Quick Start

## monoDrive C++ Client Quick Start

The monoDrive C++ Client is Open Source Software for connecting to and 
configuring the monoDrive Simulator and Scenario Editor. To get started, 
clone the client from the monoDrive repository:

```bash
$ git clone git@github.com:monoDriveIO/monodrive-client.git
```

## Setup

The monoDrive C++ Client is cross-platform and tested on both Windows 10 and 
Ubuntu 18.04.

### Windows Prerequisites

- Windows 10
- [Visual Studio 2019 Community Edition](https://visualstudio.microsoft.com/vs/community/)
- [VSCode](https://code.visualstudio.com/)
- [Boost 1.65.1](https://sourceforge.net/projects/boost/files/boost-binaries/1.65.1/boost_1_65_1-msvc-14.1-64.exe/download)
    *Note: Use the default install location*
 
### Ubuntu 18.04 Prerequisites
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04.4/)
- [VSCode](https://code.visualstudio.com/)

The packages necessary for building the client can be installed with the 
following:

```bash
$ sudo apt-get install libboost-dev \
    build-essentials \
    libeigen3-dev
```

### VSCode Build Instructions

On both Windows and Ubuntu, the C++ client can be compiled and installed in 
Visual Studio Code:

1. Open VSCode.

1. Add the following VSCode extensions:
    - CMake
    - CMake Tools
    - C/C++

1. Select `File -> Open Folder` and navigate to this folder to build the cpp-examples or simulator-cpp-client to build just the client library.

1. Use the CMake extension to configure and build:

    1. Click the Configure "All Projects" icon: 

        <p class="img_container">
        <img class="sm_img" src="../imgs/configure.png">
        </p>

    1. If prompted to Scan for Kits select "Yes":

        **Windows**: `Visual Studio Community 2019 Release - amd64`
    
        **Linux**: Choose the compiler of your choice, tested with `g++ 7.5.0`
    
    1. Build the client by clicking the `Build All Projects` icon:
    
        <p class="img_container">
        <img class="sm_img" src="../imgs/build.png">
        </p>
