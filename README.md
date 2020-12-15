# ABB EGM EXAMPLES
This repository use [abb_libegm](https://github.com/ros-industrial/abb_libegm) without [ROS](https://www.ros.org) for interact with RobostStudio.


## Table of Contents

* [Getting started](#getting-started)
  * [Dependencies](#dependencies)
  * [Building from source](#building-from-source)
* [VSCode](#vscode)
* [Author](#author)
* [License](#license)


---
## Getting Started
This project is running in a Docker container and and I use the VSCode Remote-containers extension to develop within it.


## Dependencies
Install [Docker](https://www.docker.com) and [VSCode](https://code.visualstudio.com) with [Remote-Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

## Setup and building
1. Create Project 
    - Create "Solution with Station and Virtual Controller"
        - use: IRB_1100_4kg_0.47m
        - use: RobotWare 7.0.4
        - add "Customize options"
    - In the "Change Options" windows select in "Engineering Tools": "3124-1 Externally Guided Motion (EGM)

2. Configure controller
    - go to Controller tab -> Configuration -> Communication 
    - go to "UDP Unicast Device" and add:
        - Name=EGM_REMOTE_SENSOR
        - type=UDPUC
        - RemoteAddress=REMOTE_IP
        - RemotePortNumber=6510
        - LocalPortNumber=0
    - go to "UDP Unicast Device" and add:
        - Name=EGM_LOCAL_SENSOR
        - type=UDPUC
        - RemoteAddress=HOST_IP
        - RemotePortNumber=6510
        - LocalPortNumber=0
    - restart controller

3. Add Rapid code
    - go to RAPID tab
    - open the Module1 and insert in it the code present here in RAPID/Module1.mod
    - for "CONST string EGM_SENSOR" select EGM_LOCAL_SENSOR if RobotStudio runs on the same machine as the container else select EGM_REMOTE_SENSOR

4. Open container
    - open with VSCode this repository and use "Remote-Containers:Reopen in container"

5. Building inside the container
    ~~~
    mkdir -p build
    cd build
    cmake ..
    make
    ~~~



---
## VSCode
I used these extensions:
- **c/c++** by microsoft
- **c/c++ snippets** by harsh
- **c++ intellisense** by austin
- **cmake** by twxs
- **clang-format** by xaver
- **doxgen documentation** by christoph schlosser
- **remote - containers** by microsoft
- **docker** by microsoft
- **git graph** by mhutchie
- **gruvbox mirror** by adamsome
- **vscode-icons** by icons for visual studio code


---
## Author
**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq).

Note: the [abb_libegm](https://github.com/ros-industrial/abb_libegm) library is used in this project.


## License
This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details