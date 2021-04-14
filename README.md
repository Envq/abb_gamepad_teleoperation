# **ABB GAMEPAD TELEOPERATION**
This repository use [abb_libegm](https://github.com/ros-industrial/abb_libegm) without [ROS](https://www.ros.org) for interact with RobostStudio.

The example files of the libegm have been reported from which we started to create a wrapper aimed at carrying out teleoperation. As a final result it was possible to carry out teleoperation in position using a gamepad.

**Note:** 
- It was not possible to test with the real robot.
- docker integration is present, but i could not read gamepad values from windows, so you need to install SDL2 to teleoperate with gamepad.
- Tested on: master in Macos, slave in Windows.

![telop](doc/teleopABB.gif)[teloperation]


---
## **Table of Contents**

* [Getting started](#getting-started)
  * [Dependencies](#dependencies)
  * [Building from source](#building-from-source)
* [Examples](#examples)
    * [abb_examples/](#abb_examples)
    * [raw_controller](#raw_controller)
    * [simple_controller](#simple_controller)
    * [test_controller](#test_controller)
    * [joystick_controller](#joystick_controller)
* [VSCode](#vscode)
* [Author](#author)
* [License](#license)


---
## **Getting Started**
This project is running in a Docker container and and I use the VSCode Remote-containers extension to develop within it.

**Note:** docker integration is present, but i could not read gamepad values from windows, so you need to install SDL2 to teleoperate with gamepad.


---
## **Dependencies**
Install [Docker](https://www.docker.com) and [VSCode](https://code.visualstudio.com) with [Remote-Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

To use without Docker install:
- Boost
- Protobuf
- SDL2


---
## **Setup and building**
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

5. Building inside the container (you can specify if you want to compile examples)
    ~~~
    mkdir -p build
    cd build
    cmake .. -DBUILD_EXAMPLES=false
    make
    ~~~


---
## **Examples**
All examples are launchable inside docker apart from joystick_controller. 

A class (simple_interface) that wraps around abb_libegm has been created that simplifies its use, added some utilities (custom_exception, custom_utils) and a class (joystick_interface) to be launched on the host to read the joystick values using [SDL2](https://www.libsdl.org/download-2.0.php).


### **abb_examples/**
These are the examples found [here](https://github.com/ros-industrial/abb_libegm/issues/18) cleaned up by ROS

Use this for compile also abb_examples
~~~
cmake .. -DBUILD_EXAMPLES=true
~~~


### **raw_controller**
This is my refactor of abb_examples/pose_controller.cpp

### **simple_controller**
This example is identical to raw_controller but implemented with simple_interface

### **test_controller**
This example runs some tests that can be called up with the relative number per argument:
~~~
1 -> REPLANNING
2 -> FEEDBACK with one send
3 -> FEEDBACK with more send
4 -> CONVERGENCE
5 -> WORKSPACE
~~~


### **joystick_controller**
This example performs teleoperation using a joystick (tested with logitech f310).

Since on macos and windows I can't access the joystick from inside the docker container, joystick_interface and consequently joystick_controller are only executable from the host. Additional installation of [SDL2](https://www.libsdl.org/download-2.0.php) is required.

For Execute joystick task in a cube of 25mm (default is 50mm) from initial pose use it: 

~~~
./joystick_controller 25
~~~
Commands:
- Button START -> exit from task
- JoystickL and JoystickR -> control end effector position relative to the initial pose.
- Button LB -> set Origin
- Button B -> increment strength
- Button X -> decrement strength

**Note:** default strenght is 50 mm. (this value represents the maximum distance that can be reached by moving the joystick)


---
## **VSCode**
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
## **Author**
**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq).

**Note:** the [abb_libegm](https://github.com/ros-industrial/abb_libegm) library is used in this project.


## **License**
This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details